//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic
#define WEAK_END

#include "os/module.h"
#include "core/gpio.h"
#include "os/Signal.h"
#include "com/msgs/EventMessage.hpp"
#include "com/stack_com/ArdCom.hpp"
#include "components/log/log.h"
#include "components/led/led.h"
#include "components/robot/power.h"
#include "master/uiMiddleware.h"
#include "os/os.h"
#include "end.h"
#include "match_time.h"

using namespace arp_stm32;

#define END_STACK_SIZE           1024
uint32_t end_match_duration = 0; //!< duree du match en ms

static void end_task(void *arg);
volatile bool end_match;
volatile bool begin_match;
Signal endSignal;

static int end_module_init()
{
	end_match = 0;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(end_task, "end", END_STACK_SIZE, NULL, PRIORITY_TASK_END, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	return MODULE_INIT_SUCCESS;
}

module_init(end_module_init, INIT_END);

uint32_t end_get_match_time_togo()
{
	return end_match_duration - systick_get_match_time().ms;
}

static void end_task(void *arg)
{
	(void) arg;

	//Blocking call until the math starts
	ui_waitForMatchStart();

	begin_match = 1;

	//Propagate event to master
	EventMessage msgBegin(EVT_INFORM_START_MATCH);
	ArdCom::getInstance().send(msgBegin);

	//Inform user
	ui_matchRuning();

	//If a match duration is set, wait until the timeout
	if( end_match_duration )
	{
		portTickType lastWakeTime = systick_get_match_begin_tickcount();
		portTickType periodInMs = ms_to_tick(end_match_duration);
		vTaskDelayUntil( &lastWakeTime, periodInMs );
	}
	//else wait a signal to trigger the match end
	else
	{
		endSignal.wait();
	}

	end_match = 1;

	//Propagate event to master
	EventMessage msgEnd(EVT_INFORM_END_MATCH);
	ArdCom::getInstance().send(msgEnd);

	//Inform user
	ui_matchEnded();

	power_set(POWER_OFF_END_MATCH);
	log(LOG_INFO, "Fin du match");

	exitModules();
	vTaskSuspend(0);
}

bool isMatchEnded()
{
	return end_match;
}

bool isMatchBegun()
{
	return begin_match;
}
