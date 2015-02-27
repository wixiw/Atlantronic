/*
 * ArdCom_c_wrapper.h
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */

#ifndef USB_ARD_H_
#define USB_ARD_H_

#include <stdint.h>
#include "com/msgs/DiscoveryIpcTypes.h"
#include "com/usb/circular_buffer.h"
#include "components/log/log.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*EventCallback)(void);

//used by usb read task to deserialize a received message
int usb_received_buffer_CB(CircularBuffer* const buffer);

//Send the version string as a bootup message, this message should be automatically sent at bootup
//used by usb_write task
void sendBootup();

//Subscribe to let "fct" be the event "id" message callback
void registerEventCallback(EventId id, EventCallback fct);

//Has to be called during the usb module init in order to register
//message callbacks
void usb_ard_init();

//Call to send a preformated message on the usb bus
//used by logging framework.
void usb_add_log(enum log_level, const char* func, uint16_t line, const char* msg);

//is true when the x86 board has configured the stm32, which states that the communication is established.
bool isX86Connected();

//is true when the x86 is ready to begin self tests
bool isX86ReadyForSelfTest();

//is true when the x86 is reading to begin the match
bool isX86ReadyForMatch();

#ifdef __cplusplus
}
#endif

#endif /* USB_ARD_H_ */
