/*
 * usb_cb.h
 *
 *  Created on: Feb 22, 2015
 *      Author: willy
 */

#ifndef USB_CB_H_
#define USB_CB_H_

#include "driver_ST/usbd_req.h"
#include "usb_descriptor.h"

//user callbacks required by  ST driver
uint8_t usbd_atlantronic_init(void* pdev , uint8_t cfgidx);
uint8_t usbd_atlantronic_deinit(void* pdev , uint8_t cfgidx);
uint8_t usbd_atlantronic_setup(void* pdev , USB_SETUP_REQ* req);
uint8_t* usbd_atlantronic_get_config(uint8_t speed, uint16_t *length);


//--------------------------------

extern uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];
extern uint8_t USBD_OtherSpeedCfgDesc[USB_LEN_CFG_DESC];
extern uint8_t USBD_LangIDDesc[USB_DEVICE_DESCRIPTOR_SIZE];
extern USBD_DEVICE USR_desc;

uint8_t *     USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ManufacturerStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ProductStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_USR_USRStringDesc (uint8_t speed, uint8_t idx , uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */

#endif /* USB_CB_H_ */
