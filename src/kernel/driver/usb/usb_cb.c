/*
 * usb_cb.c
 *
 *  Created on: Feb 22, 2015
 *      Author: willy
 */

#include "usb_cb.h"
#include "usb_descriptor.h"
#include "kernel/driver/usb/stm32f4xx/usbd_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_def.h"

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

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



