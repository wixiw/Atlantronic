#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_USB
#include "core/asm/asm_base_func.h"
#define WEAK_USB __attribute__((weak, alias("nop_function") ))
#endif

#include "os/portmacro.h"

//Send a message on usb, you have to call getUsbMutex() before
void usb_write(const void* buffer, int size) WEAK_USB;

//Hold the lock on the usb stack to prevent mixing when sending a message
void take_txUsbMutex() WEAK_USB;

//Release the lock on the usb stack after having called takeUsbMutex
void release_txUsbMutex() WEAK_USB;

//Signal the writing task that a message is ready, you shall NOT use this from an interrupt context
void signal_txUsbMsg() WEAK_USB;


//TODO a enlever
enum
{
	USB_DETECTION_DYNAMIC_OBJECT_SIZE1 = 7,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE2 = 8,
	USB_DETECTION_DYNAMIC_OBJECT1 = 9,
	USB_DETECTION_DYNAMIC_OBJECT2 = 10,
	USB_DATA_MAX = 12,     //!< nombre d'id, laisser en dernier
};



#ifdef __cplusplus
}
#endif

#endif
