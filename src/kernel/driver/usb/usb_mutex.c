/*
 * usb_mutex.c
 *
 *  Created on: Feb 22, 2015
 *      Author: willy
 */

#define WEAK_USB

#include "usb.h"
#include "kernel/FreeRTOS.h"
#include "kernel/semphr.h"

static xSemaphoreHandle usb_tx_mutex;
static xSemaphoreHandle usb_tx_sem;

int create_txUsbMutex()
{
	usb_tx_mutex = xSemaphoreCreateMutex();

	if(usb_tx_mutex == NULL)
		return 0;
	else
		return 1;
}

int create_txUsbSignal()
{
	vSemaphoreCreateBinary(usb_tx_sem);
	xSemaphoreTake(usb_tx_sem, 0);

	if(usb_tx_sem == NULL)
		return 0;
	else
		return 1;
}

void take_txUsbMutex()
{
	xSemaphoreTake(usb_tx_mutex, portMAX_DELAY);
}

void release_txUsbMutex()
{
	xSemaphoreGive(usb_tx_mutex);
}

void signal_txUsbMsg()
{
	xSemaphoreGive(usb_tx_sem);
}

void signal_txUsbMsgFromIsr(portBASE_TYPE* xHigherPriorityTaskWoken)
{
	xSemaphoreGiveFromISR(usb_tx_sem, xHigherPriorityTaskWoken);
}

void wait_txUsbMsg()
{
	xSemaphoreTake(usb_tx_sem, portMAX_DELAY);
}

