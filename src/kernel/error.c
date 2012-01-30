//! @file error.c
//! @brief Error codes
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/log.h"
#include "kernel/module.h"
#include "kernel/error.h"
#include "kernel/driver/usb.h"
#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"

#define ERROR_QUEUE_SIZE       50
#define ERROR_STACK_SIZE       70

struct error_item
{
	unsigned int id;
	unsigned char state;
	uint64_t time;
};

struct error_status error_status[ERR_MAX];

static void error_task(void* arg);

static xQueueHandle error_queue;

static int error_module_init(void)
{
	int i = 0;
	for( ; i < ERR_MAX; i++)
	{
		error_status[i].state = 0;
		error_status[i].time = 0;
	}

	error_queue = xQueueCreate(ERROR_QUEUE_SIZE, sizeof(struct error_item));

	if(error_queue == NULL)
	{
		return ERR_INIT_ERROR;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(error_task, "error", ERROR_STACK_SIZE, NULL, PRIORITY_TASK_ERROR, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN;
	}

	return 0;
}

module_init(error_module_init, INIT_ERROR);

static void error_task(void* arg)
{
	(void) arg;

	struct error_item error;

	usb_add(USB_ERR, error_status, sizeof(error_status));

	while(1)
	{
		if(xQueueReceive(error_queue, &error, portMAX_DELAY))
		{
			if(error_status[error.id].state != error.state)
			{
				error_status[error.id].state = error.state;
				error_status[error.id].time = error.time;
				usb_add(USB_ERR, error_status, sizeof(error_status));
			}
		}
	}
}

void error_check_update(enum fault id, uint32_t err)
{
	if(id == err)
	{
		error(id, ERROR_ACTIVE);
	}
	else
	{
		error(id, ERROR_CLEAR);
	}
}

void error(enum fault id, unsigned char new_state)
{
	struct error_item err;
	err.id = id;
	err.state = new_state;
	err.time = systick_get_time();
	xQueueSendToBack(error_queue, &err, portMAX_DELAY);
}

long error_from_isr(enum fault id, unsigned char new_state)
{
	long xHigherPriorityTaskWoken;
	struct error_item err;
	err.id = id;
	err.state = new_state;
	err.time = systick_get_time_from_isr();
	if(xQueueSendToBackFromISR(error_queue, &err, &xHigherPriorityTaskWoken) != pdPASS)
	{
		// erreur, file pleine
		// TODO a voir
	}

	return xHigherPriorityTaskWoken;
}