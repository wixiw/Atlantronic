/*
 * master.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#include "components/robot/power.h"
#include "com/stack_com/heartbeat.h"
#include "os/module.h"
#include "os/os.h"
#include "fsm.h"
#include "master.h"
#include "end.h"
#include "uiMiddleware.h"
#include "ArdCom_c_wrapper.h"
#include "selftests.hpp"

#define MASTER_STACK_SIZE    800
#define MASTER_PERIOD_MS	 100

unsigned int master_waitStartIn_transition (unsigned int currentState)
{
	if(ui_isStartPlugged())
	{
		return MASTER_STATE_WAITING_COLOR_CHOICE;
	}
	else
	{
		return currentState;
	}
}

void master_waitColor_entry()
{
	eMatchColor color = ui_requestMatchColor();
	setColor(color);
}

unsigned int master_waitAu_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( power_isEmergencyStopFired() )
	{
		return MASTER_STATE_WAIT_AU_UP;
	}
	else
	{
		return MASTER_STATE_STM32_SELF_TESTS;
	}
}

unsigned int master_waitEndSelfTestStm32_transition (unsigned int currentState)
{
	if( isSelftestFinished() )
	{
		return MASTER_STATE_WAITING_SELF_TEST_X86;
	}
	else
	{
		return currentState;
	}
}

unsigned int master_beginSelfTestX86_transition (unsigned int currentState)
{
	if( isX86Connected() )
	{
		return MASTER_STATE_X86_SELF_TESTING;
	}
	else
	{
		return currentState;
	}
}

unsigned int master_endSelfTestX86_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( isX86ReadyForMatch() )
	{
		return MASTER_STATE_WAITING_START_PLUG;
	}
	else
	{
		return currentState;
	}
}

unsigned int master_waitingMatchStartIn_transition (unsigned int currentState)
{
	if(ui_isStartPlugged())
	{
		return MASTER_STATE_WAITING_MATCH_BEGIN;
	}
	else
	{
		return currentState;
	}
}


unsigned int master_waitingMatch_transition (unsigned int currentState)
{
	if( isMatchBegun() )
	{
		return MASTER_STATE_MATCH_RUNNING;
	}
	else
	{
		return currentState;
	}
}

unsigned int master_matchRunning_transition (unsigned int currentState)
{
	if( isMatchEnded() )
	{
		return MASTER_STATE_MATCH_ENDED;
	}
	else
	{
		return currentState;
	}
}


static StateMachineState master_states[] = {
		{ "MASTER_STATE_WAITING_START_IN",
				&ui_ubiquityWaitingStartIn,
				&no_run,
				&no_exit,
				&master_waitStartIn_transition},
		{ "MASTER_STATE_WAITING_COLOR_CHOICE",
				&master_waitColor_entry,
				&no_run,
				&no_exit,
				&master_waitAu_transition},
		{ "MASTER_STATE_WAIT_AU_UP",
				&ui_displayEmergencyStopActive,
				&no_run,
				&no_exit,
				&master_waitAu_transition },
		{ "MASTER_STATE_STM32_SELF_TESTS",
				&ui_selfTesting,
				&selftests_run,
				&no_exit,
				&master_waitEndSelfTestStm32_transition },
		{ "MASTER_STATE_WAITING_SELF_TEST_X86",
				&ui_ubiquityBooting,
				&no_run,
				&no_exit,
				&master_beginSelfTestX86_transition},
		{ "MASTER_STATE_X86_SELF_TESTING",
				&ui_selfTesting,
				&no_run,
				&no_exit,
				&master_endSelfTestX86_transition},
		{ "MASTER_STATE_WAITING_START_PLUG",
				&ui_ubiquityReadyForMatch,
				&no_run,
				&no_exit,
				&master_waitingMatchStartIn_transition},
		{ "MASTER_STATE_WAITING_MATCH_BEGIN",
				&ui_ubiquityWaitForMatch,
				&no_run,
				&no_exit,
				&master_waitingMatch_transition},
		{ "MASTER_STATE_MATCH_RUNNING",
				&ui_matchRuning,
				&no_run,
				&no_exit,
				&master_matchRunning_transition},
		{ "MASTER_STATE_MATCH_ENDED",
				&ui_matchEnded,
				&no_run,
				&no_exit,
				&no_transition}
};
static StateMachine master_fsm(master_states, MASTER_STATE_MAX);

void master_sendFeedback()
{

	//TODO

//
//#include "core/gpio.h"
//#include "com/msgs/StatusMessage.hpp"
//#include "com/stack_com/ArdCom.hpp"
//#include "components/gyro/gyro.h"
//#include "components/localization/location.h"
//#include "components/log/fault.h"
//#include "components/robot/power.h"
//#include "match_time.h"
//#include "end.h"
//using namespace arp_stm32;

	//static struct control_usb_data control_usb_data;

	//if(configured)
//	control_usb_data.current_time = systick_get_time();
//	control_usb_data.match_time_elapsed = systick_get_match_time().ms;
//	control_usb_data.match_time_togo = end_get_match_time_togo();
//	control_usb_data.raw_data_gyro = gyro_get_raw_data();
//	control_usb_data.gpio = gpio_get_state();
//	control_usb_data.omega_gyro = gyro_get_omega();
//	control_usb_data.pos_theta_gyro_euler = gyro_get_theta_euler();
//	control_usb_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
//	control_usb_data.vBat = adc_filtered_data.vBat;
//	control_usb_data.iPwm[0] = adc_filtered_data.i[0];
//	control_usb_data.iPwm[1] = adc_filtered_data.i[1];
//	control_usb_data.iPwm[2] = adc_filtered_data.i[2];
//	control_usb_data.iPwm[3] = adc_filtered_data.i[3];
//	control_usb_data.pumpState = pump_update();
//	//TODO control_usb_data.color = (uint8_t) getColor();
//	control_usb_data.power_state = power_get();
//	dynamixel_update_usb_data(&control_usb_data.dynamixel);
//
//	//Send message on usb com stack
//	StatusMessage msg(control_usb_data);
//	ArdCom::getInstance().send(msg);
}



void master_task(void* arg)
{
	UNUSED(arg);

	portTickType lastWakeTime;
	const portTickType periodInTicks = ms_to_tick(MASTER_PERIOD_MS);

	//On laisse un peu de temps à tout le monde pour booter
	vTaskDelay(ms_to_tick(200));

	lastWakeTime = xTaskGetTickCount ();
	while(1)
	{
		// mise a jour heartbeat
		heartbeat_update();

		//Execution du step de la machine
		master_fsm.execute();

		//Send data to x86
		master_sendFeedback();

		//delai
		vTaskDelayUntil( &lastWakeTime, periodInTicks );
	}

}



static int master_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(master_task, "master", MASTER_STACK_SIZE, NULL, PRIORITY_TASK_MASTER, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_MASTER;
	}

	return MODULE_INIT_SUCCESS;
}

module_init(master_module_init, INIT_ARM);
