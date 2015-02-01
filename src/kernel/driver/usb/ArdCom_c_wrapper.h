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


typedef void (*EventCallback)(void);

typedef struct
{
	uint8_t* data; //pointer on buffer start
	unsigned int size; 	//total size of the buffer
	unsigned int start; //pointer on data begin
	unsigned int end;   //pointer on data end +1 = next place to write
	unsigned int count; //nbdata currnetly in buffer = end - start - 1 % size

} CircularBuffer;

int deserialize_ard(CircularBuffer const * const buffer);
void registerEventCallback(EventId id, EventCallback fct);
void usb_ard_init();
unsigned char usb_is_get_version_done();




#ifdef __cplusplus
}
#endif

#endif /* USB_ARD_H_ */
