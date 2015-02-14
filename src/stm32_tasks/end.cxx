//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "led.h"
#include "gpio.h"
#include "kernel/driver/power.h"
#include "ipc_disco/EventMessage.hpp"
#include "kernel/driver/usb/ArdCom.h"
#include "discovery/Signal.h"

using namespace arp_stm32;

#define END_STACK_SIZE           1024
uint32_t end_match_time = 0; //!< duree du match en ms

static void end_task(void *arg);
volatile int end_match;
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

	return 0;
}

module_init(end_module_init, INIT_END);

void end_cmd_set_time(uint32_t time)
{
	if( ! getGo() )
	{
		end_match_time = time;

		if( end_match_time )
			log_format(LOG_INFO, "duree du match => %d ms", (int)end_match_time);
		else
			log_format(LOG_INFO, "duree du match => no limit");
	}
}

void end_quit_match()
{
	endSignal.set();
}

static void end_task(void *arg)
{
	(void) arg;

	gpio_wait_go();
	EventMessage msgBegin(EVT_INFORM_START_MATCH);
	ArdCom::getInstance().send(msgBegin);

	if( end_match_time )
	{
		vTaskDelay(end_match_time);
	}
	else
	{
		endSignal.wait();
	}

	end_match = 1;

	setLed(0x00);
	EventMessage msgEnd(EVT_INFORM_END_MATCH);
	ArdCom::getInstance().send(msgEnd);

	power_set(POWER_OFF_END_MATCH);
	log(LOG_INFO, "Fin du match");

	exitModules();
	vTaskSuspend(0);
}
