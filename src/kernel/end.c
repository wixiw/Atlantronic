//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "gpio.h"

#define END_STACK_SIZE           100
const uint64_t DUREE_MATCH_TICK = 90ULL * 72000000ULL;

static void end_task(void *arg);

static int end_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(end_task, "end", END_STACK_SIZE, NULL, PRIORITY_TASK_END, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	return 0;
}

module_init(end_module_init, INIT_END);

static void end_task(void *arg)
{
	(void) arg;

//	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);
	while(getGo() == 0)
	{
		vTaskDelay(ms_to_tick(50));
	}

	vTaskSetEvent(EVENT_GO);
	vTaskDelay(DUREE_MATCH_TICK);
	vTaskSetEvent(EVENT_END);

	exitModules();
	setLed(0x00);

	vTaskDelete(NULL);
}