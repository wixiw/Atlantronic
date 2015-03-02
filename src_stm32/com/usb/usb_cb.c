/*
 * usb_cb.c
 *
 *  Created on: Feb 22, 2015
 *      Author: willy
 */

#include "usb_cb.h"
#include "driver_ST/usbd_core.h"
#include "driver_ST/usbd_def.h"

uint8_t usbd_atlantronic_init(void* pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	DCD_EP_Open(pdev, USB_TX_EP_ADDR, 0x40, USB_OTG_EP_BULK);
	DCD_EP_Open(pdev, USB_RX_EP_ADDR, 0x40, USB_OTG_EP_BULK);

	return USBD_OK;
}

uint8_t usbd_atlantronic_deinit(void* pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	DCD_EP_Close(pdev, USB_TX_EP_ADDR);
	DCD_EP_Close(pdev, USB_RX_EP_ADDR);

	return USBD_OK;
}

uint8_t usbd_atlantronic_setup(void* pdev , USB_SETUP_REQ* req)
{
	(void) pdev;
	(void) req;
	return USBD_OK;
}

uint8_t* usbd_atlantronic_get_config(uint8_t speed, uint16_t *length)
{
	(void) speed;
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

//------------------------------------------------------------

/**
* @brief  USBD_USR_DeviceDescriptor
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t * USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	*length = sizeof(usb_device_descriptor);
	return (uint8_t*) usb_device_descriptor;
}

/**
* @brief  USBD_USR_LangIDStrDescriptor
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	*length =  sizeof(usb_string_langID);
	return (uint8_t*)usb_string_langID;
}


/**
* @brief  USBD_USR_ProductStrDescriptor
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_PRODUCT_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ManufacturerStrDescriptor
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

/**
* @brief  USBD_USR_SerialStrDescriptor
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_SERIALNUMBER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ConfigStrDescriptor
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


/**
* @brief  USBD_USR_InterfaceStrDescriptor
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_INTERFACE_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}


