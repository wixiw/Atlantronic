#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"

#include "kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_usr.h"
#include "kernel/driver/usb/stm32f4xx/usbd_desc.h"
#include "kernel/driver/usb/stm32f4xx/usb_dcd_int.h"
#include "gpio.h"

#include "kernel/driver/usb.h"
#include "boot_signals.h"
#include "kernel/driver/usb/ArdCom_c_wrapper.h"

// Attention, pour l'envoi de commandes par usb, on suppose que c'est envoyé en une seule trame usb

#define USB_TX_BUFER_SIZE       8192
#define USB_RX_BUFER_SIZE       4096 //Take care to accord with MSG_MAX_SIZE in IpcTypes.hpp
#define USB_READ_STACK_SIZE      800
#define USB_WRITE_STACK_SIZE     800

// variables statiques => segment bss, initialisation a 0

static unsigned char usb_buffer[USB_TX_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static int usb_buffer_size;
static unsigned char usb_rx_buffer_ep[64]; //!< buffer usb de reception d'un endpoint si on n'a pas 64 octets contigus pour le mettre directement dans le buffer circulaire
static unsigned char usb_rx_buffer[USB_RX_BUFER_SIZE]; //!< buffer usb de reception (circulaire)
static unsigned int usb_rx_buffer_head; //!< position ou on doit ajouter les nouveaux octets
static unsigned int usb_rx_buffer_tail; //!< position ou on doit lire les octets
static volatile unsigned int usb_rx_buffer_count; //!< taille du buffer usb de reception
static unsigned int usb_rx_buffer_ep_used;
static int usb_rx_waiting; //!< overflow sur usb - reception. Vaut 1 si on doit relancer la reception
static xSemaphoreHandle usb_mutex;

void usb_read_task(void *);
void usb_write_task(void *);

static xSemaphoreHandle usb_write_sem;
static xSemaphoreHandle usb_read_sem;
static volatile unsigned int usb_endpoint_ready;
static __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
void USB_OTG_BSP_mDelay (const uint32_t msec);

void takeUsbMutex()
{
	xSemaphoreTake(usb_mutex, portMAX_DELAY);
}

void releaseUsbMutex()
{
	xSemaphoreGive(usb_mutex);
}

void signalUsbMsg()
{
	xSemaphoreGive(usb_write_sem);
}

static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	gpio_pin_init(GPIOA,  9, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
	gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_OD, GPIO_PUPD_UP);     // ID
	gpio_pin_init(GPIOA, 11, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOA, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	gpio_af_config(GPIOA,  9, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 10, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 11, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 12, GPIO_AF_OTG_FS);
	USBD_Init(&USB_OTG_dev, 1, &USR_desc, &USBD_atlantronic_cb, &USR_cb);
	DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	vSemaphoreCreateBinary(usb_write_sem);
	if( usb_write_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_write_sem, 0);

	vSemaphoreCreateBinary(usb_read_sem);
	if( usb_read_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_read_sem, 0);

	portBASE_TYPE err = xTaskCreate(usb_read_task, "usb_r", USB_READ_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	err = xTaskCreate(usb_write_task, "usb_w", USB_WRITE_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	usb_ard_init();

	return 0;
}

module_init(usb_module_init, INIT_USB);

void usb_write_byte(unsigned char byte)
{
	usb_buffer[usb_buffer_end] = byte;
	usb_buffer_end = (usb_buffer_end + 1) % USB_TX_BUFER_SIZE;
	usb_buffer_size++;

	if( usb_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_buffer_size = USB_TX_BUFER_SIZE;
		usb_buffer_begin = usb_buffer_end;
	}
}

void usb_write(const void* buffer, int size)
{
	int nMax = USB_TX_BUFER_SIZE - usb_buffer_end;

	usb_buffer_size += size;

	if( likely(size <= nMax) )
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, size);
		usb_buffer_end = (usb_buffer_end + size) % USB_TX_BUFER_SIZE;
	}
	else
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, nMax);
		size -= nMax;
		memcpy(&usb_buffer[0], buffer + nMax, size);
		usb_buffer_end = size;
	}

	if( usb_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_buffer_size = USB_TX_BUFER_SIZE;
		usb_buffer_begin = usb_buffer_end;
	}
}

//! Usb read task
void usb_read_task(void * arg)
{
	(void) arg;

	while(1)
	{
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_rx_buffer_count )
		{
			CircularBuffer circularBuffer; //TODO refactorer le driver pour utiliser ça partout
			portENTER_CRITICAL();
			circularBuffer.data = usb_rx_buffer;
			circularBuffer.size = USB_RX_BUFER_SIZE;
			circularBuffer.start = usb_rx_buffer_tail;
			circularBuffer.end = usb_rx_buffer_head;
			circularBuffer.count = usb_rx_buffer_count;
			portEXIT_CRITICAL();

			int totalRead = 0;

			while( circularBuffer.count > 0)
			{
				int readBytes = deserialize_ard(&circularBuffer);
				if( readBytes > 0 )
				{
					circularBuffer.start += readBytes;
					circularBuffer.count -= readBytes;
					totalRead += readBytes;
				}
				else
				{
					break;
				}
			}

			portENTER_CRITICAL();
			__sync_sub_and_fetch(&usb_rx_buffer_count, totalRead);
			usb_rx_buffer_tail = (usb_rx_buffer_tail + totalRead) % sizeof(usb_rx_buffer);
			portEXIT_CRITICAL();

			if( unlikely(usb_rx_waiting != 0))
			{
				int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
				int count = sizeof(usb_rx_buffer) - usb_rx_buffer_count;
				if( count < nMax )
				{
					nMax = count;
				}

				if( nMax > 0 )
				{
					// on a eu un overflow, il faut relancer la reception des messages
					DCD_EP_PrepareRx(&USB_OTG_dev, 2, &usb_rx_buffer[usb_rx_buffer_head], nMax);
					usb_rx_waiting = 0;
				}
			}

			if( usb_rx_buffer_count )
			{
				// on a traite un message et il en reste
				// on enchaine sans prendre la semaphore
				continue;
			}
		}

		xSemaphoreTake(usb_read_sem, portMAX_DELAY);
	}
}

//! Usb write task
void usb_write_task(void * arg)
{
	(void) arg;

	sendBootup();

	while(1)
	{
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_endpoint_ready )
		{
			takeUsbMutex();
			if(usb_buffer_size > 0)
			{
				int sizeMax = USB_TX_BUFER_SIZE - usb_buffer_begin;
				if(usb_buffer_size < sizeMax )
				{
					sizeMax = usb_buffer_size;
				}

				usb_endpoint_ready = 0;
				DCD_EP_Tx(&USB_OTG_dev, 0x81, usb_buffer + usb_buffer_begin, sizeMax);
				usb_buffer_size -= sizeMax;
				usb_buffer_begin = (usb_buffer_begin + sizeMax) % USB_TX_BUFER_SIZE;
			}
			releaseUsbMutex();
		}

		xSemaphoreTake(usb_write_sem, portMAX_DELAY);
	}
}

void isr_otg_fs(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void EP1_IN_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( ! usb_endpoint_ready )
	{
		usb_endpoint_ready = 1;
		xSemaphoreGiveFromISR(usb_write_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void EP2_OUT_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	// pas de commande en cours de traitement
	int count = USBD_GetRxCount(&USB_OTG_dev, 0x02);
	if( unlikely(usb_rx_buffer_ep_used) )
	{
		int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
		if( count <= nMax )
		{
			memcpy(&usb_rx_buffer[usb_rx_buffer_head], usb_rx_buffer_ep, count);
		}
		else
		{
			memcpy(&usb_rx_buffer[usb_rx_buffer_head], usb_rx_buffer_ep, nMax);
			memcpy(usb_rx_buffer, &usb_rx_buffer_ep[nMax], count - nMax);
		}
	}
	usb_rx_buffer_ep_used = 0;
	usb_rx_buffer_count += count;
	usb_rx_buffer_head = (usb_rx_buffer_head + count) % sizeof(usb_rx_buffer);

	int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
	count = sizeof(usb_rx_buffer) - usb_rx_buffer_count;
	if( count < nMax )
	{
		nMax = count;
	}

	if( likely(nMax >= 64) )
	{
		DCD_EP_PrepareRx(&USB_OTG_dev, 2, &usb_rx_buffer[usb_rx_buffer_head], nMax);
	}
	else if( count > 64)
	{
		DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer_ep, nMax);
		usb_rx_buffer_ep_used = 1;
	}
	else
	{
		// overflow
		usb_rx_waiting = 1;
	}

	xSemaphoreGiveFromISR(usb_read_sem, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}


