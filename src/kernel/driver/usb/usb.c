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
#include <stdint.h>

//
// STM32 => outdoor
//
#define USB_TX_BUFER_SIZE       8192
#define USB_TX_STACK_SIZE     	800
static uint8_t usb_tx_buffer[USB_TX_BUFER_SIZE];
static int usb_tx_buffer_begin;
static int usb_tx_buffer_end;
static int usb_tx_buffer_size;
static xSemaphoreHandle usb_tx_mutex;
static xSemaphoreHandle usb_tx_sem;
void usb_write_task(void *);

//
// OUTDOOR => STM32
//
#define USB_RX_BUFER_SIZE        200
#define USB_RX_CIRCULAR_SIZE     512  //Take care to accord with MSG_MAX_SIZE in IpcTypes.hpp //TODO a reaugmenter apres debug
#define USB_RX_STACK_SIZE      	 800
static uint8_t usb_rx_circular_buffer[USB_RX_CIRCULAR_SIZE]; //private buffer should not be accessed directly
static uint8_t usb_rx_tmp_buffer[USB_RX_BUFER_SIZE]; 		 // buffer de reception du endpoint il est utilisé par le HW
static CircularBuffer usb_rx_buffer;      					 // buffer (normal) usb de reception, il est mis à jour par l'IT avec usb_rx_tmp_buffer
static xSemaphoreHandle usb_rx_sem;
void usb_read_task(void *);


static volatile unsigned int usb_endpoint_ready;
static __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
void USB_OTG_BSP_mDelay (const uint32_t msec);


void take_txUsbMutex()
{
	xSemaphoreTake(usb_tx_mutex, portMAX_DELAY);
}

void release_txUsbMutex()
{
	xSemaphoreGive(usb_tx_mutex);
}

void signal_txUsbMsg()
{
	xSemaphoreGive(usb_tx_sem);
}


static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	//mapping de la structure de gestion du buffer circulaire sur le buffer de reception usb
	circular_create( &usb_rx_buffer, usb_rx_circular_buffer, USB_RX_CIRCULAR_SIZE);

	usb_tx_mutex = xSemaphoreCreateMutex();

	if(usb_tx_mutex == NULL)
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
	DCD_EP_PrepareRx(&USB_OTG_dev, USB_RX_EP_ADDR, usb_rx_tmp_buffer, sizeof(usb_rx_tmp_buffer));

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	vSemaphoreCreateBinary(usb_tx_sem);
	if( usb_tx_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_tx_sem, 0);

	vSemaphoreCreateBinary(usb_rx_sem);
	if( usb_rx_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_rx_sem, 0);

	portBASE_TYPE err = xTaskCreate(usb_read_task, "usb_rx", USB_RX_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	err = xTaskCreate(usb_write_task, "usb_tx", USB_TX_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

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
	usb_tx_buffer[usb_tx_buffer_end] = byte;
	usb_tx_buffer_end = (usb_tx_buffer_end + 1) % USB_TX_BUFER_SIZE;
	usb_tx_buffer_size++;

	if( usb_tx_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_tx_buffer_size = USB_TX_BUFER_SIZE;
		usb_tx_buffer_begin = usb_tx_buffer_end;
	}
}

void usb_write(const void* buffer, int size)
{
	int nMax = USB_TX_BUFER_SIZE - usb_tx_buffer_end;

	usb_tx_buffer_size += size;

	if( likely(size <= nMax) )
	{
		memcpy(&usb_tx_buffer[usb_tx_buffer_end], buffer, size);
		usb_tx_buffer_end = (usb_tx_buffer_end + size) % USB_TX_BUFER_SIZE;
	}
	else
	{
		memcpy(&usb_tx_buffer[usb_tx_buffer_end], buffer, nMax);
		size -= nMax;
		memcpy(&usb_tx_buffer[0], buffer + nMax, size);
		usb_tx_buffer_end = size;
	}

	if( usb_tx_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_tx_buffer_size = USB_TX_BUFER_SIZE;
		usb_tx_buffer_begin = usb_tx_buffer_end;
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

		while( circular_getOccupiedRoom(&usb_rx_buffer) > 0)
		{
			portENTER_CRITICAL();
			int readBytes = deserialize_ard(&usb_rx_buffer);
			portEXIT_CRITICAL();

			if( readBytes <= 0 )
			{
				//The buffer is not empty, but it doesn't contain a complete datagram
				break;
			}
		}

		xSemaphoreTake(usb_rx_sem, portMAX_DELAY);
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
			take_txUsbMutex();
			if(usb_tx_buffer_size > 0)
			{
				int sizeMax = USB_TX_BUFER_SIZE - usb_tx_buffer_begin;
				if(usb_tx_buffer_size < sizeMax )
				{
					sizeMax = usb_tx_buffer_size;
				}

				usb_endpoint_ready = 0;
				DCD_EP_Tx(&USB_OTG_dev, USB_TX_EP_ADDR, usb_tx_buffer + usb_tx_buffer_begin, sizeMax);
				usb_tx_buffer_size -= sizeMax;
				usb_tx_buffer_begin = (usb_tx_buffer_begin + sizeMax) % USB_TX_BUFER_SIZE;
			}
			release_txUsbMutex();
		}

		xSemaphoreTake(usb_tx_sem, portMAX_DELAY);
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
		xSemaphoreGiveFromISR(usb_tx_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void EP2_OUT_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	int rxDataSize = USBD_GetRxCount(&USB_OTG_dev, USB_RX_EP_ID);
	if( !circular_append(&usb_rx_buffer, usb_rx_tmp_buffer, rxDataSize))
	{
		//s'il n'y a plus de place dans le buffer tournant c'est la merde
		while(1);
	}

	//
	//Relancement de la lecture des buffers USB :
	//
	memset(usb_rx_tmp_buffer, 0, sizeof(usb_rx_tmp_buffer));
	DCD_EP_PrepareRx(&USB_OTG_dev, USB_RX_EP_ADDR, usb_rx_tmp_buffer, sizeof(usb_rx_tmp_buffer));
	xSemaphoreGiveFromISR(usb_rx_sem, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}


