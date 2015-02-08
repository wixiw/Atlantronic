/*
 * ArdCom_c_wrapper.h
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */

#ifndef USB_ARD_H_
#define USB_ARD_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <stdint.h>
#include "ipc_disco/DiscoveryIpcTypes.h"
#include "kernel/log.h"

typedef void (*EventCallback)(void);

typedef struct
{
	uint8_t* data; //pointer on buffer start
	unsigned int size; 	//total size of the buffer
	unsigned int start; //pointer on data begin
	unsigned int end;   //pointer on data end +1 = next place to write
	unsigned int count; //nbdata currnetly in buffer = end - start - 1 % size

} CircularBuffer;

//used by usb read task to deserialize a received message
int deserialize_ard(CircularBuffer const * const buffer);

//Subscribe to let "fct" be the event "id" message callback
void registerEventCallback(EventId id, EventCallback fct);

//Has to be called during the usb module init in order to register
//message callbacks
void usb_ard_init();

//Call to send a preformated message on the usb bus
//used by logging framework.
void usb_add_log(enum log_level, const char* func, uint16_t line, const char* msg);

//is non null when the x86 board has requested the version, which mean that the communication is established.
//used by led task
unsigned char usb_is_get_version_done();

#ifdef __cplusplus
}
#endif

#endif /* USB_ARD_H_ */
