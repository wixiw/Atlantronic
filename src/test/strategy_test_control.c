#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control/control.h"
#include "time2.h"
#include "event.h"
#include "io/gpio.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       64

static void strategy_test_control_task();
int strategy_test_control_module_init();

int strategy_test_control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strategy_test_control_task, (const signed char *) "strategy_test_control", STRATEGY_TEST_CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(strategy_test_control_module_init, INIT_STRATEGY);

static void strategy_test_control_task()
{
	time_start_match();

	if(getColor() == COLOR_BLUE)
	{
		control_straight(1800);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1000);
//		vTaskWaitEvent(EVENT_CONTROL_READY);
//		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(200);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);

		vTaskWaitEvent(EVENT_CONTROL_READY);
	}
	else
	{
/*		control_straight(500);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);*/
	}
/*	control_goto(1000,-1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_goto(1500,-1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
*/
	vTaskDelete(NULL);
}
