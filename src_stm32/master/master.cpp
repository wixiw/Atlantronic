/*
 * master.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#include "components/power/power.h"
#include "core/module.h"
#include "master/fsm.h"
#include "os/os.h"
#include "master.h"
#include "end.h"
#include "uiMiddleware.h"
#include "ArdCom_c_wrapper.h"

#define MASTER_STACK_SIZE    300
#define MASTER_PERIOD_MS	 100

unsigned int master_waitAu_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( power_isEmergencyStopFired() )
	{
		return MASTER_STATE_WAIT_AU_UP;
	}
	else
	{
		if(isX86Connected())
		{
			return MASTER_STATE_UNCONFIGURED;
		}
		else
		{
			return MASTER_STATE_DEPLOYING;
		}
	}
}

unsigned int master_deployed_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( isX86ReadyForSelfTest())
	{
		return MASTER_STATE_WAITING_SELF_TEST_BEGIN;
	}
	else
	{
		return MASTER_STATE_DEPLOYING;
	}
}

unsigned int master_beginSelfTest_transition (unsigned int currentState)
{
	UNUSED(currentState);
	ui_waitForSelfTestStart();
	return MASTER_STATE_SELF_TESTING;
}

unsigned int master_endSelfTest_transition (unsigned int currentState)
{
	UNUSED(currentState);
	//TODO verifier que les tests sont faits
	return MASTER_STATE_PLACING_ROBOT_FOR_MATCH;
}

unsigned int master_placingRobot_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( isX86ReadyForMatch() )
	{
		return MASTER_STATE_WAITING_MATCH_BEGIN;
	}
	else
	{
		return MASTER_STATE_PLACING_ROBOT_FOR_MATCH;
	}
}


unsigned int master_waitingMatch_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( isMatchBegun() )
	{
		return MASTER_STATE_MATCH_RUNNING;
	}
	else
	{
		return MASTER_STATE_WAITING_MATCH_BEGIN;
	}
}

unsigned int master_matchRunning_transition (unsigned int currentState)
{
	UNUSED(currentState);
	if( isMatchEnded() )
	{
		return MASTER_STATE_MATCH_ENDED;
	}
	else
	{
		return MASTER_STATE_MATCH_RUNNING;
	}
}

void master_waitColor_entry()
{
	eMatchColor color = ui_requestMatchColor();
	setColor(color);
	//TODO safety color avec retour strat x86 ?
}


static StateMachineState master_states[MASTER_STATE_MAX] = {
	/*	{ "MASTER_STATE_WAITING_COLOR_CHOICE",
				&master_waitColor_entry,
				&no_run,
				&master_waitAu_transition},
		{ "MASTER_STATE_WAIT_AU_UP",
				&ui_displayEmergencyStopActive,
				&no_run,
				&master_waitAu_transition },
		{ "MASTER_STATE_UNCONFIGURED",
				&ui_ubiquityBooting,
				&no_run,
				&master_waitAu_transition },
		{ "MASTER_STATE_DEPLOYING",
				&ui_ubiquityBooting,
				&no_run,
				&master_deployed_transition},
		{ "MASTER_STATE_WAITING_SELF_TEST_BEGIN",
				&ui_ubiquityReadyForSelfTests,
				&no_run,
				&master_beginSelfTest_transition},
		{ "MASTER_STATE_SELF_TESTING",
				&ui_selfTesting,
				&no_run,
				&master_endSelfTest_transition},
		{ "MASTER_STATE_PLACING_ROBOT_FOR_MATCH",
				&ui_selfTesting,
				&no_run,
				&master_placingRobot_transition},
		{ "MASTER_STATE_WAITING_MATCH_BEGIN",
				&ui_ubiquityReadyForMatch,
				&no_run,
				&master_waitingMatch_transition},
		{ "MASTER_STATE_MATCH_RUNNING",
				&ui_matchRuning,
				&no_run,
				&master_matchRunning_transition},
		{ "MASTER_STATE_MATCH_ENDED",
				&ui_matchEnded,
				&no_run,
				&no_transition},*/
};
static StateMachine master_fsm(master_states, MASTER_STATE_MAX);



void master_task(void* arg)
{
	UNUSED(arg);

	portTickType lastWakeTime;
	const portTickType periodInMs = ms_to_tick(MASTER_PERIOD_MS);

	while(1)
	{
		master_fsm.execute();
		vTaskDelayUntil( &lastWakeTime, periodInMs );
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
