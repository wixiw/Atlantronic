#define WEAK_USB
#include "priority.h"

#include "usb.h"
#include "usb_cb.h"
#include "usb_descriptor.h"

#include "driver_ST/usb_dcd_int.h"
#include "driver_ST/usbd_core.h"
#include "driver_ST/usbd_ioreq.h"
#include "driver_ST/usbd_usr.h"

#include "os/os.h"
#include "os/module.h"
#include "core/gpio.h"
#include "master/ArdCom_c_wrapper.h"


#include <stdint.h>

//
// STM32 => outdoor
//
#define USB_TX_BUFER_SIZE       4096
#define USB_TX_STACK_SIZE     	800
static uint8_t usb_tx_buffer[USB_TX_BUFER_SIZE];
static int usb_tx_buffer_begin;
static int usb_tx_buffer_end;
static int usb_tx_buffer_size;
void usb_write_task(void *);
int create_txUsbMutex();
int create_txUsbSignal();
void wait_txUsbMsg();
void signal_txUsbMsgFromIsr(portBASE_TYPE* xHigherPriorityTaskWoken) WEAK_USB;
static volatile unsigned int usb_endpoint_ready;
uint8_t usb_cb_DataIn(void* pdev , uint8_t epnum); //Attention le IN est veut dire TX, c'est in pour stm32=>OTG


//
// OUTDOOR => STM32
//
#define USB_RX_TMP_BUFER_SIZE   4096
#define USB_RX_CIRCULAR_SIZE    8096
#define USB_RX_STACK_SIZE      	1500
static uint8_t usb_rx_circular_buffer[USB_RX_CIRCULAR_SIZE]; //private buffer should not be accessed directly
static uint8_t usb_rx_tmp_buffer[USB_RX_TMP_BUFER_SIZE]; 		 // buffer de reception du endpoint il est utilisé par le HW
static CircularBuffer usb_rx_buffer;      					 // buffer (normal) usb de reception, il est mis à jour par l'IT avec usb_rx_tmp_buffer
static xSemaphoreHandle usb_rx_sem;
void usb_read_task(void *);
uint8_t usb_cb_DataOut(void* pdev , uint8_t epnum); //Attention le OUT est veut dire RX, c'est in pour OTG=>stm32
extern int usb_received_buffer_CB(CircularBuffer* buffer);

//
// ST driver related
//

static __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
void USB_OTG_BSP_mDelay (const uint32_t msec);

USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor,
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,
};

 USBD_Class_cb_TypeDef USBD_atlantronic_cb =
{
	usbd_atlantronic_init,
	usbd_atlantronic_deinit,
	usbd_atlantronic_setup,
	0,
	0,
	usb_cb_DataIn,
	usb_cb_DataOut,
	0,
	0,
	0,
	usbd_atlantronic_get_config,
};



//------------------------------------------------------------


static int usb_module_init(void)
{
	//Configure USB HW
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	gpio_pin_init(GPIOA,  9, GPIO_MODE_IN, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS pas utile en mode device
    gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // ID pas utile en mode devide
	gpio_pin_init(GPIOA, 11, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOA, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	gpio_af_config(GPIOA,  9, GPIO_AF_OTG_FS); // pas utile en mode device
	gpio_af_config(GPIOA, 10, GPIO_AF_OTG_FS);// pas utile en mode devide
	gpio_af_config(GPIOA, 11, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 12, GPIO_AF_OTG_FS);

	USBD_Init(&USB_OTG_dev, 1, &USR_desc, &USBD_atlantronic_cb, &USR_cb);
	DCD_EP_PrepareRx(&USB_OTG_dev, USB_RX_EP_ADDR, usb_rx_tmp_buffer, sizeof(usb_rx_tmp_buffer));
	usb_endpoint_ready = 1;
	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	//mapping de la structure de gestion du buffer circulaire sur le buffer de reception usb
	circular_create( &usb_rx_buffer, usb_rx_circular_buffer, USB_RX_CIRCULAR_SIZE);

	if( !create_txUsbMutex() )
	{
		return ERR_INIT_USB;
	}

	if( !create_txUsbSignal() )
	{
		return ERR_INIT_USB;
	}

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

	return MODULE_INIT_SUCCESS;
}

module_init(usb_module_init, INIT_USB);

//Interrupt callback for the ST usb driver
void isr_otg_fs(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}



//--------------------------------------------//
// STM32 => outside
//--------------------------------------------//

//ARD utility function to write in the circular TX buffer
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

//ST callback to send data from stm32=>outside
uint8_t usb_cb_DataIn(void* pdev , uint8_t epnum)
{
	UNUSED(pdev);
	if( epnum == USB_TX_EP_ID)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = 0;
		portSET_INTERRUPT_MASK_FROM_ISR();

		if( ! usb_endpoint_ready )
		{
			usb_endpoint_ready = 1;
			signal_txUsbMsgFromIsr(&xHigherPriorityTaskWoken);
		}

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
	}
	return USBD_OK;
}

//! Usb write task
void usb_write_task(void * arg)
{
	UNUSED(arg);

	while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
	{
		vTaskDelay( ms_to_tick(100) );
	}

	sendBootup();

	while(1)
	{
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
		wait_txUsbMsg();
	}
}


//--------------------------------------------//
// outside => STM32
//--------------------------------------------//



//ST callback to receive data from outside=>stm32
uint8_t usb_cb_DataOut(void* pdev , uint8_t epnum)
{
	UNUSED(pdev);
	if( epnum == USB_RX_EP_ID)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = 0;
		portSET_INTERRUPT_MASK_FROM_ISR();

		uint16_t rxDataSize = USBD_GetRxCount(&USB_OTG_dev, USB_RX_EP_ID);
		if( rxDataSize > sizeof(usb_rx_tmp_buffer))
		{
			log_format(LOG_ERROR, "Can't handle such bandwith in RX :(, I'm dead.");
			return USBD_FAIL;
		}

		if( !circular_append(&usb_rx_buffer, usb_rx_tmp_buffer, rxDataSize))
		{
			//s'il n'y a plus de place dans le buffer tournant c'est la merde
			log_format(LOG_ERROR, "Can't handle such bandwith in RX :(, I'm dead.");
			return USBD_FAIL;
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

	return USBD_OK;
}


//! Usb read task
void usb_read_task(void * arg)
{
	UNUSED(arg);

	while(1)
	{
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		while( circular_getOccupiedRoom(&usb_rx_buffer) > 0)
		{
			int readBytes = usb_received_buffer_CB(&usb_rx_buffer);
			if( readBytes <= 0 )
			{
				//The buffer is not empty, but it doesn't contain a complete datagram
				break;
			}
		}

		xSemaphoreTake(usb_rx_sem, portMAX_DELAY);
	}
}



