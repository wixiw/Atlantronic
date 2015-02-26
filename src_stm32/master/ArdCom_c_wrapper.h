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

#ifndef WEAK_USB
#include "core/asm/asm_base_func.h"
#define WEAK_USB __attribute__((weak, alias("nop_function") ))
#endif

typedef void (*EventCallback)(void);

//used by usb read task to deserialize a received message
int deserialize_ard(CircularBuffer* const buffer) WEAK_USB;

//Send the version string as a bootup message, this message should be automatically sent at bootup
//used by usb_write task
void sendBootup() WEAK_USB;

//Subscribe to let "fct" be the event "id" message callback
void registerEventCallback(EventId id, EventCallback fct) WEAK_USB;

//Has to be called during the usb module init in order to register
//message callbacks
void usb_ard_init() WEAK_USB;

//Call to send a preformated message on the usb bus
//used by logging framework.
void usb_add_log(enum log_level, const char* func, uint16_t line, const char* msg) WEAK_USB;

//is true when the x86 board has configured the stm32, which states that the communication is established.
bool isX86Connected() WEAK_USB;

//is true when the x86 is ready to begin self tests
bool isX86ReadyForSelfTest() WEAK_USB;

//is true when the x86 is reading to begin the match
bool isX86ReadyForMatch() WEAK_USB;

#ifdef __cplusplus
}
#endif

#endif /* USB_ARD_H_ */
