/*
 * usb_cb.h
 *
 *  Created on: Feb 22, 2015
 *      Author: willy
 */

#ifndef USB_CB_H_
#define USB_CB_H_

#include "kernel/driver/usb/stm32f4xx/usbd_def.h"

//user callbacks provided by  ST driver
uint8_t usb_cb_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_de_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_Setup(struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
//uint8_t usb_cb_EP0_TxSent(struct _USBD_HandleTypeDef *pdev );
//uint8_t usb_cb_EP0_RxReady(struct _USBD_HandleTypeDef *pdev );
/* Class Specific Endpoints*/
//uint8_t usb_cb_SOF(struct _USBD_HandleTypeDef *pdev);
//uint8_t usb_cb_IsoINIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
//uint8_t usb_cb_IsoOUTIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t* usb_cb_GetHSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetFSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetOtherSpeedConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetDeviceQualifierDescriptor(uint16_t *length);



#endif /* USB_CB_H_ */
