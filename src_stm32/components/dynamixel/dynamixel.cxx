//! @file dynamixel.cxx
//! @brief Gestion des dynamixel (ax12 et rx24)
//! @author Atlantronic

#define WEAK_DYNAMIXEL
#include "dynamixel.h"
#include "os/module.h"
#include "os/rcc.h"
#include "com/stack_com/ArdCom.hpp"
#include "components/robot/power.h"
#include "components/log/log.h"
#include "master/end.h"

#include <stdlib.h>
#include <math.h>

#define DYNAMIXEL_INSTRUCTION_PING             0x01
#define DYNAMIXEL_INSTRUCTION_READ_DATA        0x02
#define DYNAMIXEL_INSTRUCTION_WRITE_DATA       0x03
#define DYNAMIXEL_INSTRUCTION_REG_WRITE        0x04
#define DYNAMIXEL_INSTRUCTION_ACTION           0x05
#define DYNAMIXEL_INSTRUCTION_RESET            0x06
#define DYNAMIXEL_INSTRUCTION_SYNC_WRITE       0x83

#define DYNAMIXEL_STACK_SIZE           1024
#define DYNAMIXEL_READ_TIMEOUT           15 // en ms
#define DYNAMIXEL_MOVE_TIMEOUT         3000 // en ms

static uint8_t dynamixel_checksum(uint8_t* buffer, uint8_t size);

static void dynamixel_cmd_set_id(DynamixelManager* manager, uint8_t old_id, uint8_t id);

DynamixelManager ax12;
DynamixelManager rx24;

static int dynamixel_module_init()
{
	ax12.init("ax12", USART6_HALF_DUPLEX, 200000, NB_MAX_AX12, 12);
	rx24.init("rx24", UART5_FULL_DUPLEX, 200000, NB_MAX_RX24, 24);

	return 0;
}

module_init(dynamixel_module_init, INIT_DYNAMIXEL);

int DynamixelManager::init(const char* name, enum usart_id usart_id, uint32_t frequency, int Max_devices_id, uint8_t Type)
{
	usart = usart_id;
	type = Type;
	int res = usart_open(usart, frequency);
	if( res )
	{
		return -1;
	}

	portBASE_TYPE err = xTaskCreate(task_wrapper, name, DYNAMIXEL_STACK_SIZE, this, PRIORITY_TASK_DYNAMIXEL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_DYNAMIXEL;
	}

	mutex = xSemaphoreCreateMutex();

	if( mutex == 0)
	{
		return ERR_INIT_DYNAMIXEL;
	}

	usart_mutex = xSemaphoreCreateMutex();

	if( usart_mutex == 0)
	{
		return ERR_INIT_DYNAMIXEL;
	}

	usart_set_write_dma_buffer(usart, write_dma_buffer);
	usart_set_read_dma_buffer(usart, read_dma_buffer);

	max_devices_id = Max_devices_id;
	devices = (Dynamixel*) malloc(sizeof(Dynamixel)*max_devices_id);
	int i;
	for(i = 0; i < max_devices_id-1; i++)
	{
		devices[i].last_error.transmit_error = 0xff;
		devices[i].max_goal = 0x3ff;
		devices[i].goal_pos = 0x1ff;
		devices[i].target_reached_threshold = 2;
		devices[i].flags = DYNAMIXEL_FLAG_TARGET_REACHED | DYNAMIXEL_FLAG_CONTROL_OFF | DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
	}

	return MODULE_INIT_SUCCESS;
}

void DynamixelManager::task_wrapper(void* arg)
{
	DynamixelManager* manager = (DynamixelManager*) arg;
	manager->task();
}

void DynamixelManager::task()
{
	struct dynamixel_request req;
	int id = 1;

	while(1)
	{
		// lecture de la position du dynamixel
		req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
		req.arg[0] = DYNAMIXEL_PRESENT_POSITION_L;
		req.arg[1] = 0x02;
		req.argc = 2;
		req.id = id + 1;

		// pas de log d erreur de com si pas de puissance
		if( ! power_get() )
		{
			send(&req);
		}
		else
		{
			req.status.error.transmit_error = ERR_DYNAMIXEL_POWER_OFF;
		}

		if( !req.status.error.transmit_error )
		{
			int pos = req.status.arg[0] + (req.status.arg[1] << 8);
			xSemaphoreTake(mutex, portMAX_DELAY);
			devices[id].pos = pos;
			int goal_pos = devices[id].goal_pos;

			uint16_t pos_err = abs(pos - goal_pos);
			systime t = systick_get_time();
			if( devices[id].flags & DYNAMIXEL_FLAG_CONTROL_OFF )
			{
				devices[id].timeStartMoving_ms = t.ms;
			}
			if( pos_err <= devices[id].target_reached_threshold)
			{
				devices[id].flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
				devices[id].timeStartMoving_ms = t.ms;
				if( devices[id].flags & DYNAMIXEL_FLAG_STUCK )
				{
					log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), id+1);
					devices[id].flags &= ~DYNAMIXEL_FLAG_STUCK;
				}
			}
			else if( pos_err > devices[id].target_reached_threshold + 1)
			{
				if( devices[id].flags & DYNAMIXEL_FLAG_TARGET_REACHED )
				{
					devices[id].timeStartMoving_ms = t.ms;
					devices[id].flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
				}

				if( t.ms - devices[id].timeStartMoving_ms > DYNAMIXEL_MOVE_TIMEOUT )
				{
					if( ! (devices[id].flags & DYNAMIXEL_FLAG_STUCK) )
					{
						log_format(LOG_ERROR, "%s id %d stucked", pcTaskGetTaskName(NULL), id+1);
						devices[id].flags |= DYNAMIXEL_FLAG_STUCK;
					}
				}
				else if( devices[id].flags & DYNAMIXEL_FLAG_STUCK )
				{
					log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), id+1);
					devices[id].flags &= ~DYNAMIXEL_FLAG_STUCK;
				}
			}

			// renvoi du couple uniquement si ! control_off et torque_to_update
			if( (devices[id].flags & (DYNAMIXEL_FLAG_TORQUE_TO_UPDATE | DYNAMIXEL_FLAG_CONTROL_OFF)) == DYNAMIXEL_FLAG_TORQUE_TO_UPDATE || disabled)
			{
				uint16_t max_torque = 0;
				if( ! disabled )
				{
					max_torque = devices[id].max_torque;
				}
				struct dynamixel_error err = write16(id+1, DYNAMIXEL_TORQUE_LIMIT_L, max_torque);
				if( ! err.transmit_error && ! err.internal_error )
				{
					err = write8(id+1, DYNAMIXEL_TORQUE_ENABLE, 0x01);
					if( ! err.transmit_error )
					{
						devices[id].flags &= ~DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
					}
				}
			}
			xSemaphoreGive(mutex);

			if( pos_err > 0 && ! (devices[id].flags & DYNAMIXEL_FLAG_CONTROL_OFF) && ! disabled)
			{
				// on va envoyer la position désirée
				req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
				req.arg[0] = DYNAMIXEL_GOAL_POSITION_L;
				req.arg[1] = (uint8_t) (goal_pos & 0xFF);
				req.arg[2] = (uint8_t) ((goal_pos >> 8) & 0xFF);
				req.argc = 3;
				send(&req);
			}
		}

		xSemaphoreTake(mutex, portMAX_DELAY);
		if( devices[id].last_error.transmit_error )
		{
			devices[id].flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
		}
		if(devices[id].last_error.transmit_error != req.status.error.transmit_error || devices[id].last_error.internal_error != req.status.error.internal_error)
		{
			print_error(req.id, req.status.error);
			devices[id].last_error = req.status.error;
		}

		xSemaphoreGive(mutex);

		id = (id + 1) % (max_devices_id-1);
		if(id == 0)
		{
			id++;
			vTaskDelay(5);
		}
	}
}

void dynamixel_update_usb_data(struct dynamixel_usb_data* dynamixel)
{
	xSemaphoreTake(ax12.mutex, portMAX_DELAY);
	for(int i = 1; i < NB_MAX_AX12; i++)
	{
		dynamixel->ax12[i].pos = ax12.devices[i-1].pos;
		dynamixel->ax12[i].flags = ax12.devices[i-1].flags;
		dynamixel->ax12[i].error = ax12.devices[i-1].last_error;
	}
	xSemaphoreGive(ax12.mutex);

	xSemaphoreTake(rx24.mutex, portMAX_DELAY);
	for(int i = 1; i < NB_MAX_RX24; i++)
	{
		dynamixel->rx24[i].pos = rx24.devices[i-1].pos;
		dynamixel->rx24[i].flags = rx24.devices[i-1].flags;
		dynamixel->rx24[i].error = rx24.devices[i-1].last_error;
	}
	xSemaphoreGive(rx24.mutex);
}

void dynamixel_enable()
{
	ax12.disabled = false;
	rx24.disabled = false;
}

void dynamixel_disable()
{
	ax12.disabled = true;
	rx24.disabled = true;
}

void DynamixelManager::send(struct dynamixel_request *req)
{
	uint32_t res = 0;
	uint8_t write_size = 6 + req->argc;
	uint8_t read_size = 0;
	uint8_t i;
	uint8_t* buffer;

	xSemaphoreTake(usart_mutex, portMAX_DELAY);

	write_dma_buffer[0] = 0xFF;
	write_dma_buffer[1] = 0xFF;
	write_dma_buffer[2] = req->id;
	write_dma_buffer[3] = 0x02 + req->argc;
	write_dma_buffer[4] = req->instruction;
	for( i = 0; i < req->argc ; i++)
	{
		write_dma_buffer[5+i] = req->arg[i];
	}
	write_dma_buffer[write_size-1] = dynamixel_checksum(write_dma_buffer, write_size);

	read_size = write_size;

	if(req->id != 0xFE)
	{
		if(req->instruction != DYNAMIXEL_INSTRUCTION_READ_DATA)
		{
			read_size += 6;
		}
		else
		{
			read_size += 6 + req->arg[1];
		}
	}

	usart_set_read_dma_size(usart, read_size);
	usart_send_dma_buffer(usart, write_size);

	res = usart_wait_read(usart, DYNAMIXEL_READ_TIMEOUT);
	if( res )
	{
		goto end;
	}

	buffer = read_dma_buffer;
	// verification des données envoyées
	for(i = 0; i< write_size ; i++)
	{
		if( write_dma_buffer[i] != read_dma_buffer[i] )
		{
			// erreur, on n'a pas lus ce qui a été envoyé
			res = ERR_DYNAMIXEL_SEND_CHECK;
			goto end;
		}
	}

	buffer += write_size;

	// pas de broadcast => réponse attendue
	if(req->id != 0xFE)
	{
		int size = read_size - write_size;
		if(buffer[0] != 0xFF || buffer[1] != 0xFF || buffer[2] != req->id || buffer[3] != size - 4)
		{
			// erreur protocole
			res = ERR_DYNAMIXEL_PROTO;
			goto end;
		}

		if( buffer[size - 1] != dynamixel_checksum(buffer, size))
		{
			// erreur checksum
			res = ERR_DYNAMIXEL_CHECKSUM;
			goto end;
		}

		req->status.error.internal_error = buffer[4];

		if(size == 7)
		{
			req->status.arg[0] = buffer[5];
		}
		else if(size == 8)
		{
			req->status.arg[0] = buffer[5];
			req->status.arg[1] = buffer[6];
		}
	}

end:
	if(res && ! (res & ERR_USART_TIMEOUT) )
	{
		// on vide tout ce qui traine dans le buffer de reception
		usart_set_read_dma_size(usart, sizeof(read_dma_buffer));
		usart_wait_read(usart, DYNAMIXEL_READ_TIMEOUT);
	}
	xSemaphoreGive(usart_mutex);
	req->status.error.transmit_error = res;
}

static uint8_t dynamixel_checksum(uint8_t* buffer, uint8_t size)
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< size - 1 ; i++)
	{
		checksum += buffer[i];
	}
	checksum = ~checksum;

	return checksum;
}

void DynamixelManager::print_error(int id, struct dynamixel_error err)
{
	if(err.transmit_error)
	{
		if(err.transmit_error == ERR_DYNAMIXEL_SEND_CHECK)
		{
			log_format(LOG_ERROR, "%s %3d : échec de la verification des octets envoyés", pcTaskGetTaskName(NULL), id);
		}
		else if(err.transmit_error == ERR_DYNAMIXEL_PROTO)
		{
			log_format(LOG_ERROR, "%s %3d : erreur protocole", pcTaskGetTaskName(NULL), id);
		}
		else if( err.transmit_error == ERR_DYNAMIXEL_CHECKSUM)
		{
			log_format(LOG_ERROR, "%s %3d : somme de verification incompatible", pcTaskGetTaskName(NULL), id);
		}
		else if( err.transmit_error == ERR_DYNAMIXEL_POWER_OFF)
		{
			log_format(LOG_ERROR, "%s %3d : power off", pcTaskGetTaskName(NULL), id);
		}
		else
		{
			if(err.transmit_error & ERR_USART_TIMEOUT)
			{
				log_format(LOG_ERROR, "%s %3d : timeout", pcTaskGetTaskName(NULL), id);
			}
			else if(err.transmit_error & ERR_USART_READ_SR_FE)
			{
				log_format(LOG_ERROR, "%s %3d : desynchro, bruit ou octet \"break\" sur l'usart", pcTaskGetTaskName(NULL), id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_NE)
			{
				log_format(LOG_ERROR, "%s %3d : bruit sur l'usart", pcTaskGetTaskName(NULL), id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_ORE)
			{
				log_format(LOG_ERROR, "%s %3d : overrun sur l'usart", pcTaskGetTaskName(NULL), id);
			}
		}
	}
	else if(err.internal_error)
	{
		if( err.internal_error & DYNAMIXEL_INPUT_VOLTAGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - problème de tension", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_ANGLE_LIMIT_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - angle invalide", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_OVERHEATING_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - surchauffe", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_RANGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - valeur non admissible", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_CHECKSUM_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - somme de verification incompatible", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_OVERLOAD_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - surcharge de l'actioneur", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_INSTRUCTION_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - instruction invalide", pcTaskGetTaskName(NULL), id);
		}
	}
	else
	{
		log_format(LOG_INFO, "%s %3d ok", pcTaskGetTaskName(NULL), id);
	}
}

uint8_t DynamixelManager::read8(uint8_t id, uint8_t offset, struct dynamixel_error* error)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x01;
	req.argc = 2;

	send(&req);
	*error = req.status.error;

	return req.status.arg[0];
}

uint16_t DynamixelManager::read16(uint8_t id, uint8_t offset, struct dynamixel_error* error)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x02;
	req.argc = 2;

	send(&req);
	*error = req.status.error;

	return req.status.arg[0] + (req.status.arg[1] << 8);
}

struct dynamixel_error DynamixelManager::write8(uint8_t id, uint8_t offset, uint8_t data)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = data;
	req.argc = 2;

	send(&req);
	return req.status.error;
}

struct dynamixel_error DynamixelManager::write16(uint8_t id, uint8_t offset, uint16_t data)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = (uint8_t) (data & 0xFF);
	req.arg[2] = (uint8_t) ((data >> 8) & 0xFF);
	req.argc = 3;

	send(&req);
	return req.status.error;
}

struct dynamixel_error DynamixelManager::ping(uint8_t id)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_PING;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

struct dynamixel_error DynamixelManager::action(uint8_t id)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_ACTION;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

struct dynamixel_error DynamixelManager::reset(uint8_t id)
{
	struct dynamixel_request req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_RESET;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

struct dynamixel_error DynamixelManager::set_led(uint8_t id, uint8_t on)
{
	return write8(id, DYNAMIXEL_LED, on);
}

struct dynamixel_error DynamixelManager::set_moving_speed(uint8_t id, float speed)
{
	uint16_t speed16 = (uint16_t)fabsf(DYNAMIXEL_RD_TO_POS * speed);
	if( speed16 > 0x3ff )
	{
		speed16 = 0x3ff;
	}
	if( speed < 0 )
	{
		speed16 |= 1<<10;
	}

	struct dynamixel_error error = write16(id, DYNAMIXEL_MOVING_SPEED_L, speed16);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission on dynamixel id=%d", id);
	}
	return error;
}

struct dynamixel_error DynamixelManager::set_goal_position(uint8_t id, float theta)
{
	struct dynamixel_error err;
	id--;

	// on met theta dans [-M_PI ; M_PI[
	theta = fmodf(theta, 2*M_PI);
	if( theta > M_PI)
	{
		theta -= 2*M_PI;
	}
	else if(theta < -M_PI)
	{
		theta += 2*M_PI;
	}

	// passage en unité dynamixel entre 0 et 0x3ff (de -150 degre a 150 degre)
	// zero au milieu qui vaut 0x1ff
	int32_t alpha = 0x1ff + theta * DYNAMIXEL_RD_TO_POS;
	uint16_t min = 0;
	uint16_t max = 0x3ff;

	// si c'est un dynamixel connu, on regarde les limites
	// pas de mutex, c'est de la conf
	if( id < max_devices_id-1 )
	{
		min = devices[id].min_goal;
		max = devices[id].max_goal;
	}

	// saturation
	if(alpha < min)
	{
		alpha = min;
	}
	else if( alpha > max)
	{
		alpha = max;
	}

	if( id < max_devices_id-1 )
	{
		// utilisation de la tache pour la mise à jour
		xSemaphoreTake(mutex, portMAX_DELAY);
		devices[id].flags &= ~DYNAMIXEL_FLAG_CONTROL_OFF;
		devices[id].goal_pos = alpha;
		if( abs(devices[id].goal_pos - devices[id].pos) < devices[id].target_reached_threshold)
		{
			devices[id].flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
		}
		else
		{
			devices[id].flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
		}
		err = devices[id].last_error;
		xSemaphoreGive(mutex);
	}
	else
	{
		// envoi simple
		err = write16(id+1, DYNAMIXEL_GOAL_POSITION_L, (uint16_t) alpha);
		if( err.transmit_error)
		{
			log_format(LOG_ERROR, "erreur de transmission on dynamixel id=%d", id);
		}
		else
		{
			log_format(LOG_INFO, "set goal position %d - status = %#.2x", (uint16_t)alpha, err.internal_error);
		}
	}

	return err;
}

struct dynamixel_error DynamixelManager::set_torque_limit(uint8_t id, float torque_limit)
{
	struct dynamixel_error err;
	id--;

	if( isMatchEnded() || torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	if( id < max_devices_id-1 )
	{
		// utilisation de la tache pour la mise à jour
		xSemaphoreTake(mutex, portMAX_DELAY);
		devices[id].max_torque = torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT;
		devices[id].flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
		err = devices[id].last_error;
		xSemaphoreGive(mutex);
	}
	else
	{
		// envoi simple
		err = write16(id+1, DYNAMIXEL_TORQUE_LIMIT_L, torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT);
	}

	return err;
}

struct dynamixel_error DynamixelManager::set_torque_limit_eeprom(uint8_t id, float torque_limit)
{
	if( torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	return write16(id, DYNAMIXEL_MAX_TORQUE_L, torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT);
}

struct dynamixel_error DynamixelManager::set_torque_enable(uint8_t id, uint8_t enable)
{
	return write8(id, DYNAMIXEL_TORQUE_ENABLE, enable & 0x01);
}

struct dynamixel_error DynamixelManager::set_cw_angle_limit(uint8_t id, uint16_t val)
{
	struct dynamixel_error error = write16(id, DYNAMIXEL_CW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission on dynamixel id=%d", id);
	}
	else
	{
		log_format(LOG_INFO, "cw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

struct dynamixel_error DynamixelManager::set_ccw_angle_limit(uint8_t id, uint16_t val)
{
	struct dynamixel_error error = write16(id, DYNAMIXEL_CCW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission on dynamixel id=%d", id);
	}
	else
	{
		log_format(LOG_INFO, "ccw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

void DynamixelManager::set_goal_limit(uint8_t id, float min, float max)
{
	id--;
	if( id >= max_devices_id-1 )
	{
		return;
	}

	min = 0x1ff + min * DYNAMIXEL_RD_TO_POS;
	max = 0x1ff + max * DYNAMIXEL_RD_TO_POS;

	if(min < 0)
	{
		min = 0;
	}
	else if(min > 0x3ff)
	{
		min = 0x3ff;
	}

	if( max < 0)
	{
		max = 0;
	}
	else if( max > 0x3ff)
	{
		max = 0x3ff;
	}

	devices[id].min_goal = min;
	devices[id].max_goal = max;
}

void DynamixelManager::set_target_reached_threshold(uint8_t id, float threshold)
{
	id--;
	if( id >= max_devices_id-1 )
	{
		return;
	}

	devices[id].target_reached_threshold = threshold * DYNAMIXEL_RD_TO_POS;
}

bool DynamixelManager::isFlagActive(uint8_t id,  uint32_t mask)
{
	bool res = false;
	id--;
	if(id < max_devices_id-1 )
	{
		xSemaphoreTake(mutex, portMAX_DELAY);
		res = devices[id].flags & mask;
		xSemaphoreGive(mutex);
	}

	return res;
}

float DynamixelManager::get_position(uint8_t id, struct dynamixel_error* error)
{
	id--;
	int32_t alpha = 0x1ff;
	if(id < max_devices_id-1 )
	{
		xSemaphoreTake(mutex, portMAX_DELAY);
		alpha = devices[id].pos;
		*error = devices[id].last_error;
		xSemaphoreGive(mutex);
	}
	else
	{
		alpha = read16(id, DYNAMIXEL_PRESENT_POSITION_L, error);
	}

	// passage en rd
	// zero au milieu qui vaut "0x1ff"
	return (alpha - 0x1ff) * DYNAMIXEL_POS_TO_RD;
}

__OPTIMIZE_SIZE__ void dynamixel_cmd(struct dynamixel_cmd_param const * const param)
{
	struct dynamixel_error err;
	float theta;

	DynamixelManager* manager = NULL;

	switch(param->type)
	{
		case DYNAMIXEL_TYPE_AX12:
			manager = &ax12;
			break;
		case DYNAMIXEL_TYPE_RX24:
			manager = &rx24;
			break;
		default:
			log_format(LOG_ERROR, "unknown dynamixel type : %d", param->type);
			return;
			break;
	}

	switch(param->cmd_id)
	{
			break;
		case DYNAMIXEL_CMD_SET_ID:
			dynamixel_cmd_set_id(manager, param->id, ((int)param->param) & 0xff);
			break;
		case DYNAMIXEL_CMD_SET_GOAL_POSITION:
			manager->set_goal_position(param->id, param->param);
			break;
		case DYNAMIXEL_CMD_SET_SPEED:
			manager->set_moving_speed(param->id, param->param);
			break;
		case DYNAMIXEL_CMD_SET_BAUDRATE:
			// on les met a 200kb, commande usb pour la configuration
			manager->write8(param->id, DYNAMIXEL_BAUD_RATE, 9);
			break;
		case DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE:
			usart_set_frequency(manager->usart, (uint32_t)(param->param));
			break;
		case DYNAMIXEL_CMD_SET_MAX_TORQUE:
			manager->set_torque_enable(param->id, 1);
			manager->set_torque_limit(param->id, param->param);
			break;
		case DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD:
			manager->set_target_reached_threshold(param->id, param->param);
			break;
		case DYNAMIXEL_CMD_GET_POSITION:
			theta = manager->get_position(param->id, &err);
			if(!err.transmit_error)
			{
				theta *= 180 / M_PI;
				log_format(LOG_INFO, "ax%d %u - pos %d", param->type, param->id, (int)theta);
			}
			break;
		case DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE:
			manager->set_cw_angle_limit(param->id, 0);
			manager->set_ccw_angle_limit(param->id, 0);
			break;
		case DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE:
			manager->set_cw_angle_limit(param->id, 0);
			manager->set_ccw_angle_limit(param->id, 0x3ff);
			break;
		default:
			log_format(LOG_ERROR, "unknown dynamixel command : %d", param->cmd_id);
			break;
	}
}

void dynamixel_cmd_scan()
{
	int i;
	struct dynamixel_error error;

	log(LOG_INFO, "dynamixel - scan");

	for(i = 1; i<254; i++)
	{
		error = ax12.ping(i);
		if(! error.transmit_error)
		{
			log_format(LOG_INFO, "dynamixel type %d id %3d détecté - status %#.2x", ax12.getType(), i, error.internal_error);
		}

		error = rx24.ping(i);
		if(! error.transmit_error)
		{
			log_format(LOG_INFO, "dynamixel type %d id %3d détecté - status %#.2x", rx24.getType(), i, error.internal_error);
		}
	}

	log(LOG_INFO, "dynamixel - end scan");
}

static void dynamixel_cmd_set_id(DynamixelManager* manager, uint8_t old_id, uint8_t id)
{
	struct dynamixel_error error;

	if(id <= 0xfd)
	{
		error = manager->write8(old_id, DYNAMIXEL_ID, id);
		if( error.transmit_error)
		{
			log_format(LOG_ERROR, "erreur de transmission on dynamixel id=%d", id);
		}
		else
		{
			log_format(LOG_INFO, "id %d -> %d - status = %#.2x", old_id, id, error.internal_error);
		}
	}
	else
	{
		log(LOG_ERROR, "changement d'id par broadcast non autorisé");
	}
}
