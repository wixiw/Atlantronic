#define WEAK_USB
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"

#include "kernel/driver/usb/stm32f4xx/usbd_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_def.h"
#include "kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.h"
#include "usb_descriptor.h"
#include "gpio.h"

#include "kernel/driver/usb.h"
#include "boot_signals.h"
#include "kernel/driver/usb/ArdCom_c_wrapper.h"
#include <stdint.h>

//Configuration specifique pour le stm32 ARD
uint8_t* usb_get_device_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_lang_id_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_manufacturer_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_product_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_serial_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_configuration_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_interface_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);

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
static USBD_HandleTypeDef usb_handle;
static USBD_DescriptorsTypeDef usb_descriptors =
{
	usb_get_device_descriptor,
	usb_get_lang_id_str_descriptor,
	usb_get_manufacturer_str_descriptor,
	usb_get_product_str_descriptor,
	usb_get_serial_str_descriptor,
	usb_get_configuration_str_descriptor,
	usb_get_interface_str_descriptor,
};

//user callbacks provided by  ST driver
uint8_t usb_cb_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_de_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_Setup(struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
//uint8_t usb_cb_EP0_TxSent(struct _USBD_HandleTypeDef *pdev );
//uint8_t usb_cb_EP0_RxReady(struct _USBD_HandleTypeDef *pdev );
/* Class Specific Endpoints*/
uint8_t usb_cb_DataIn(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t usb_cb_DataOut(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
//uint8_t usb_cb_SOF(struct _USBD_HandleTypeDef *pdev);
//uint8_t usb_cb_IsoINIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
//uint8_t usb_cb_IsoOUTIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t* usb_cb_GetHSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetFSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetOtherSpeedConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetDeviceQualifierDescriptor(uint16_t *length);

static USBD_ClassTypeDef usb_cb =
{
		usb_cb_init,
		usb_cb_de_init,
		usb_cb_Setup,
		NULL,//usb_cb_EP0_TxSent,
		NULL,//usb_cb_EP0_RxReady,
		usb_cb_DataIn,
		usb_cb_DataOut,
		NULL,//usb_cb_SOF,
		NULL,//usb_cb_IsoINIncomplete,
		NULL,//usb_cb_IsoOUTIncomplete,
		usb_cb_GetHSConfigDescriptor,
		usb_cb_GetFSConfigDescriptor,
		usb_cb_GetOtherSpeedConfigDescriptor,
		usb_cb_GetDeviceQualifierDescriptor,
};

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

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
    gpio_pin_init(GPIOA, 9, GPIO_MODE_IN, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
    //TODO old version :	gpio_pin_init(GPIOA,  9, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
	gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // ID
	//TODO old versions : 	gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_OD, GPIO_PUPD_UP);     // ID
	gpio_pin_init(GPIOA, 11, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOA, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	gpio_af_config(GPIOA,  9, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 10, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 11, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 12, GPIO_AF_OTG_FS);
	USBD_Init(&usb_handle, &usb_descriptors, 1);
	USBD_RegisterClass(&usb_handle, &usb_cb);
	USBD_Start(&usb_handle);
	USBD_LL_PrepareReceive(&usb_handle, USB_RX_EP_ADDR, usb_rx_tmp_buffer, sizeof(usb_rx_tmp_buffer));

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

uint8_t* usb_get_device_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	*length = sizeof(usb_device_descriptor);
	return (uint8_t*)usb_device_descriptor;
}

uint8_t* usb_get_lang_id_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	*length =  sizeof(usb_string_langID);
	return (uint8_t*)usb_string_langID;
}

uint8_t* usb_get_manufacturer_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_product_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_PRODUCT_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_serial_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_SERIALNUMBER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_configuration_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_interface_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_INTERFACE_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t usb_cb_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_BULK, 0x40);
	USBD_LL_OpenEP(pdev, 0x02, USBD_EP_TYPE_BULK, 0x40);
	return USBD_OK;
}

uint8_t usb_cb_de_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	USBD_LL_CloseEP(pdev, 0x81);
	USBD_LL_CloseEP(pdev, 0x02);
	return USBD_OK;
}

uint8_t usb_cb_Setup(struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{
	(void) pdev;
	(void) req;
	return USBD_OK;
}

//ST callback to send data from stm32=>outside
uint8_t usb_cb_DataIn(struct _USBD_HandleTypeDef *pdev , uint8_t epnum)
{
	UNUSED(pdev);
	if( epnum == USB_TX_EP_ID)
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
	return USBD_OK;
}

//ST callback to receive data from outside=>stm32
uint8_t usb_cb_DataOut(struct _USBD_HandleTypeDef *pdev , uint8_t epnum)
{
	UNUSED(pdev);
	if( epnum == USB_RX_EP_ID)
	{
	
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	int rxDataSize = USBD_GetRxCount(&usb_handle, USB_RX_EP_ID);
	if( !circular_append(&usb_rx_buffer, usb_rx_tmp_buffer, rxDataSize))
	{
		//s'il n'y a plus de place dans le buffer tournant c'est la merde
		return USBD_BUSY;
	}

	//
	//Relancement de la lecture des buffers USB :
	//
	memset(usb_rx_tmp_buffer, 0, sizeof(usb_rx_tmp_buffer));
	USBD_LL_PrepareReceive(&usb_handle, USB_RX_EP_ADDR, usb_rx_tmp_buffer, sizeof(usb_rx_tmp_buffer));
	xSemaphoreGiveFromISR(usb_rx_sem, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
	
	}
	
	return USBD_OK;
}

uint8_t* usb_cb_GetHSConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetFSConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetOtherSpeedConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetDeviceQualifierDescriptor(uint16_t *length)
{
	*length = sizeof(usb_device_qualifier_desc);
	return usb_device_qualifier_desc;
}

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
		while( usb_handle.dev_state != USBD_STATE_CONFIGURED )
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
		while( usb_handle.dev_state != USBD_STATE_CONFIGURED )
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
				USBD_LL_Transmit(&usb_handle, USB_TX_EP_ADDR, usb_tx_buffer + usb_tx_buffer_begin, sizeMax);
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
	HAL_PCD_IRQHandler(usb_handle.pData);
}



