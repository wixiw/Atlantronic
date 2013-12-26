//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Atlantronic

#include "kernel/driver/hokuyo.h"
#include "kernel/driver/usart.h"
#include "kernel/rcc.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/module.h"
#include "kernel/location/location.h"
#include <string.h>

#define ERR_HOKUYO_TIMEOUT              0x01
#define ERR_HOKUYO_USART_FE             0x02
#define ERR_HOKUYO_USART_NE             0x04
#define ERR_HOKUYO_USART_ORE            0x08
#define ERR_HOKUYO_CHECK_CMD            0x10

#define ERR_HOKUYO_UNKNOWN_STATUS       0x20
#define ERR_HOKUYO_CHECKSUM             0x40
#define ERR_HOKUYO_BAUD_RATE            0x80
#define ERR_HOKUYO_LASER_MALFUNCTION   0x100

//!< taille de la réponse maxi avec hokuyo_scan_all :
//!< 682 points => 1364 data
//!< 1364 data = 21 * 64 + 20 data
//!< donc 23 octets entête, + 21*(64+2) + (20+2) + 1 = 1432
#define HOKUYO_SCAN_BUFFER_SIZE       1432
#define HOKUYO_STACK_SIZE              400
#define HOKUYO_SPEED                750000

const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_hs_cmd = "HS0\n";
//const char* hokuyo_hs_cmd = "HS1\n";
const char* hokuyo_laser_on_cmd = "BM\n";
const char* hokuyo_scan_all = "GS0044072500\n";

class hokuyo
{
	public:
		int init(enum usart_id id, const char* name, int usb_id);
		void setPosition(VectPlan pos, int sens);
		hokuyo_callback callback;

	protected:
		static void task_wrapper(void* arg);
		void task();
		uint32_t wait_decode_scan();
		uint32_t init_com();
		uint32_t scip2();
		uint32_t transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout);
		uint32_t check_cmd(unsigned char* cmd, uint32_t size);
		uint32_t check_sum(uint32_t start, uint32_t end);
		uint32_t set_speed();
		uint32_t hs();
		uint32_t laser_on();
		int decode_scan();
		void fault_update(uint32_t err);
		void start_scan();

		static uint16_t decode16(const unsigned char* data);

		enum usart_id usartId;
		int usb_id;
		xSemaphoreHandle scan_mutex;
		uint8_t read_dma_buffer[HOKUYO_SCAN_BUFFER_SIZE];
		struct hokuyo_scan scan;
};

static struct hokuyo hokuyo[HOKUYO_MAX];

int hokuyo_module_init()
{
	int err = hokuyo[0].init(UART4_FULL_DUPLEX, "hokuyo1", USB_HOKUYO1);
	if(err)
	{
		goto done;
	}

	err = hokuyo[1].init(USART2_FULL_DUPLEX, "hokuyo2", USB_HOKUYO2);

	if(err)
	{
		goto done;
	}

	hokuyo[0].setPosition(VectPlan(0,0,0), 1);
	hokuyo[1].setPosition(VectPlan(0,0,0), 1);

done:
	return err;
}

module_init(hokuyo_module_init, INIT_HOKUYO);

int hokuyo::init(enum usart_id id, const char* name, int usbId)
{
	usartId = id;
	usb_id = usbId;
	callback = NULL;

	portBASE_TYPE err = xTaskCreate(task_wrapper, name, HOKUYO_STACK_SIZE, this, PRIORITY_TASK_HOKUYO, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	scan_mutex = xSemaphoreCreateMutex();
	if( ! scan_mutex )
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

void hokuyo::setPosition(VectPlan pos, int sens)
{
	scan.pos_hokuyo = pos;
	scan.sens = sens;
}

void hokuyo_register(enum hokuyo_id hokuyo_id, hokuyo_callback callback)
{
	if( hokuyo_id < HOKUYO_MAX )
	{
		hokuyo[hokuyo_id].callback = callback;
	}
}

void hokuyo::fault_update(uint32_t err)
{
	// TODO
	if(err)
	{
		//fault(FAULT_HOKUYO_DISCONNECTED, FAULT_ACTIVE);
		//log(LOG_INFO, "hokuyo disconnected");
	}
	else
	{
		//log(LOG_INFO, "hokuyo connected");
		//fault(FAULT_HOKUYO_DISCONNECTED, FAULT_CLEAR);
	}

	if( err & (ERR_HOKUYO_USART_FE | ERR_HOKUYO_USART_NE | ERR_HOKUYO_USART_ORE | ERR_HOKUYO_CHECK_CMD | ERR_HOKUYO_CHECKSUM | ERR_HOKUYO_UNKNOWN_STATUS))
	{
		//log(LOG_INFO, "hokuyo data corruption");
		//fault(FAULT_HOKUYO_DATA_CORRUPTION, FAULT_ACTIVE);
	}
	else
	{
		//fault(FAULT_HOKUYO_DATA_CORRUPTION, FAULT_CLEAR);
	}
}

uint32_t hokuyo::init_com()
{
	uint32_t err = 0;

	log(LOG_INFO, "Initialisation du hokuyo ...");

	do
	{
		// tentative a la vitesse d'utilisation (hokuyo déjà configuré)
		usart_open(usartId, HOKUYO_SPEED);
		usart_set_read_dma_buffer(usartId, read_dma_buffer);

		// on vide tout ce qui traine sur la ligne de reception
		while(err != ERR_USART_TIMEOUT)
		{
			usart_set_read_dma_size(usartId, HOKUYO_SCAN_BUFFER_SIZE);
			err = usart_wait_read(usartId, ms_to_tick(100));
		}

		err = scip2();
		if(err & ERR_HOKUYO_TIMEOUT)
		{
			// pas de réponse à HOKUYO_SPEED, le hokuyo n'est peut être pas configuré
			// => on tente à la vitesse de base 19200
			usart_set_frequency(usartId, 19200);

			err = scip2();

			if(err)
			{
				fault_update(err);
				continue;
			}

			log(LOG_INFO, "hokuyo - set speed");
			// mise a la bonne vitesse
			err = set_speed();

			if(err)
			{
				fault_update(err);
				continue;
			}

			usart_set_frequency(usartId, HOKUYO_SPEED);
		}

		if(err)
		{
			fault_update(err);
			continue;
		}

		err = laser_on();

		if(err)
		{
			fault_update(err);
			continue;
		}

		err = hs();

		fault_update(err);
	}
	while(err);

	log(LOG_INFO, "Hokuyo initialisé");

	return 0;
}

void hokuyo::task_wrapper(void* arg)
{
	struct hokuyo* h = (struct hokuyo*) arg;
	h->task();
}

void hokuyo::task()
{
	uint32_t err;
	struct systime last_scan_time;
	struct systime current_time;
	init_com();

	start_scan();
	// on gruge, le premier scan est plus long
	last_scan_time = systick_get_time();
	last_scan_time.ms += 100;

	while(1)
	{
		// on attend la fin du nouveau scan
		err = wait_decode_scan();
		scan.pos_robot = location_get_position(); // TODO voir si meilleur m oment
		fault_update(err);
		if(err)
		{
			init_com();
			// on gruge, le premier scan est plus long
			last_scan_time = systick_get_time();
			last_scan_time.ms += 100;
		}
		else
		{
			// on a un scan toutes les 100ms, ce qui laisse 100ms pour faire le calcul sur l'ancien scan
			// pendant que le nouveau arrive. Si on depasse les 110ms (10% d'erreur), on met un log
			current_time = systick_get_time();
			if( current_time.ms - last_scan_time.ms > 110 )
			{
				log_format(LOG_ERROR, "slow cycle : %lu ms", current_time.ms - last_scan_time.ms);
			}
			last_scan_time = current_time;
		}

		// on lance le prochain scan avant de faire les calculs sur le scan actuel
		start_scan();

		// si le dernier scan n'a pas echoue on reveille la tache detection
		if( ! err)
		{
			if(callback)
			{
				callback();
			}

			// on envoi les donnees par usb pour le debug
			usb_add(usb_id, &scan, sizeof(scan));
		}
	}
}

//! Vérifie que la commande envoyée est bien renvoyée par le hokuyo
//! @return 0 si c'est bon
//! @return ERR_HOKUYO_CHECK_CMD sinon
uint32_t hokuyo::check_cmd(unsigned char* cmd, uint32_t size)
{
	uint32_t res = 0;
	uint32_t i = 0;

	for(; i < size; i++)
	{
		if(cmd[i] != read_dma_buffer[i])
		{
			res = ERR_HOKUYO_CHECK_CMD;
			goto end;
		}
	}

end:
	return res;
}

uint32_t hokuyo::check_sum(uint32_t start, uint32_t end)
{
	uint8_t sum = 0;
	uint32_t err = 0;

	for(; start < end; start++)
	{
		sum += read_dma_buffer[start];
	}

	sum &= 0x3F;
	sum += 0x30;

	if(sum != read_dma_buffer[end])
	{
		err = ERR_HOKUYO_CHECKSUM;
	}

	return err;
}

//! Envoi une commande, attend la reponse du hokuyo, vérifie si le hokuyo fait bien un echo de la commande et le checksum du status
//! @return 0 si ok
uint32_t hokuyo::transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout)
{
	uint32_t err = 0;

	usart_set_write_dma_buffer(usartId, buf);

	usart_set_read_dma_size(usartId, read_size);
	usart_send_dma_buffer(usartId, write_size);
	err = usart_wait_read(usartId, timeout);

	if(err)
	{
		if(err & ERR_USART_TIMEOUT)
		{
			err |= ERR_HOKUYO_TIMEOUT;
		}
		if(err & ERR_USART_READ_SR_FE)
		{
			err |= ERR_HOKUYO_USART_FE;
		}
		if(err & ERR_USART_READ_SR_NE)
		{
			err |= ERR_HOKUYO_USART_NE;
		}
		if(err & ERR_USART_READ_SR_ORE)
		{
			err |= ERR_HOKUYO_USART_ORE;
		}
		goto end;
	}

	err = check_cmd(buf, write_size);

	if(err)
	{
		goto end;
	}

	err = check_sum(write_size, write_size+2);

end:
	return err;
}

uint32_t hokuyo::scip2()
{
	uint32_t err = 0;

	err = transaction((unsigned char*) hokuyo_scip2_cmd, 8, 13, ms_to_tick(100));

	if(err)
	{
		goto end;
	}

	if( read_dma_buffer[8] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;	
	}

	if( read_dma_buffer[9] != '0' &&  read_dma_buffer[9] != 'E')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

end:
	return err;
}

uint32_t hokuyo::set_speed()
{
	uint32_t err = 0;

	err = transaction((unsigned char*) hokuyo_speed_cmd, 9, 14, ms_to_tick(100));

	if(err)
	{
		goto end;
	}

	if( read_dma_buffer[9] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(read_dma_buffer[10])
	{
		case '0':
		case '3':
			// OK
			break;
		case '1':
		case '2':
			err = ERR_HOKUYO_BAUD_RATE;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

end:
	return err;
}

uint32_t hokuyo::hs()
{
	uint32_t err = 0;

	err = transaction((unsigned char*) hokuyo_hs_cmd, 4, 9, ms_to_tick(100));

	if(err)
	{
		goto end;
	}

	if( read_dma_buffer[4] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	if( read_dma_buffer[5] != '0' && read_dma_buffer[5] != '2')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

end:
	return err;
}

uint32_t hokuyo::laser_on()
{
	uint32_t err = 0;

	err = transaction((unsigned char*) hokuyo_laser_on_cmd, 3, 8, ms_to_tick(100));

	if(err)
	{
		goto end;
	}

	if( read_dma_buffer[3] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(read_dma_buffer[4])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			err = ERR_HOKUYO_LASER_MALFUNCTION;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

end:
	return err;
}

void hokuyo::start_scan()
{
	usart_set_write_dma_buffer(usartId, (unsigned char*)hokuyo_scan_all);
	usart_set_read_dma_size(usartId, HOKUYO_SCAN_BUFFER_SIZE);
	usart_send_dma_buffer(usartId, 13);
}

uint32_t hokuyo::wait_decode_scan()
{
	uint32_t err = usart_wait_read(usartId, ms_to_tick(150));

	if(err)
	{
		if(err & ERR_USART_TIMEOUT)
		{
			err |= ERR_HOKUYO_TIMEOUT;
		}
		if(err & ERR_USART_READ_SR_FE)
		{
			err |= ERR_HOKUYO_USART_FE;
		}
		if(err & ERR_USART_READ_SR_NE)
		{
			err |= ERR_HOKUYO_USART_NE;
		}
		if(err & ERR_USART_READ_SR_ORE)
		{
			err |= ERR_HOKUYO_USART_ORE;
		}
		goto end;
	}

	err = check_cmd((unsigned char*)hokuyo_scan_all, 13);

	if(err)
	{
		goto end;
	}

	err = check_sum(13, 15);

	if(err)
	{
		goto end;
	}
	
	if( read_dma_buffer[13] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(read_dma_buffer[14])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			err = ERR_HOKUYO_LASER_MALFUNCTION;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

	xSemaphoreTake(scan_mutex, portMAX_DELAY);
	err = decode_scan();
	xSemaphoreGive(scan_mutex);

end:
	return err;
}

uint16_t hokuyo::decode16(const unsigned char* data)
{
	uint16_t val = *data++ - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= *data - 0x30;

	return val;
}

int hokuyo::decode_scan()
{
	const unsigned char* buffer = read_dma_buffer;
	uint16_t* distance = scan.distance;

	int j = 0;
	int i = 0;
	uint8_t sum = 0;
	int res = 0;

	// on passe l'entête
	buffer += 23;

	// traitement des pack de 64 data + sum + LF
	for( i = 21; i--; )
	{
		sum = 0;
		for( j = 32 ; j-- ; )
		{
			*distance = decode16(buffer);
			distance++;
			sum += *buffer;
			buffer++;
			sum += *buffer;
			buffer++;
		}

		sum &= 0x3F;
		sum += 0x30;

		if( sum != *buffer)
		{
			// par sécurité, on met le code d'erreur 10
			distance -= 32;
			for( j = 32 ; j-- ; )
			{
				*distance = 10;
				distance++;
			}
			res = ERR_HOKUYO_CHECKSUM;
		}
		buffer+=2;
	}

	// TODO checksum
	// traitement du reste
	for( i = 10 ; i-- ; buffer += 2)
	{
		*distance = decode16(buffer);
		distance++;
	}

	return res;
}