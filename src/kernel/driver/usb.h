#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

//Send a message on usb, you have to call getUsbMutex() before
void usb_write(const void* buffer, int size);

//Hold the lock on the usb stack to prevent mixing when sending a message
void take_txUsbMutex();

//Release the lock on the usb stack after having called takeUsbMutex
void release_txUsbMutex();

//Signal the writing task that a message is ready
void signal_txUsbMsg();

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
