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
#include "control.h"
#include "end.h"
#include "match_time.h"
#include "uiMiddleware.h"
#include "ArdCom_c_wrapper.h"
#include "selftests.hpp"
#include "core/gpio.h"
#include "core/adc.h"
#include "com/msgs/DiscoveryIpcMessage.hpp"
#include "com/stack_com/ArdCom.hpp"
#include "components/gyro/gyro.h"
#include "components/localization/location.h"
#include "components/dynamixel/dynamixel.h"
#include "components/log/fault.h"
#include "components/pump/pump.h"
#include "components/robot/power.h"
#include "core/boot_signals.h"
using namespace arp_stm32;


#define MASTER_STACK_SIZE    800
static portTickType periodInTicks = ms_to_tick(100);

static struct control_usb_data status_msg_data;
static bool x86ReadyForMatch = false;
static bool x86SentConfig = false;

bool isX86ReadyForMatch()
{
	return x86ReadyForMatch;
}

void setX86ReadyForMatch()
{
	x86ReadyForMatch = true;
}


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

void master_selftestStm32_entry()
{
	ui_selfTesting();
	power_clear(POWER_OFF);
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
	if( x86SentConfig )
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
				&master_selftestStm32_entry,
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

void master_configuration(Datagram& dtg)
{
	if( ArdCom::getInstance().isConnected() )
	{
		log_format(LOG_ERROR, "protocol error, configuration is already done.");
		return;
	}

	//Deserialize the message
	ConfigurationMsg msg;
	if( !msg.deserialize(dtg.getPayload()) )
	{
		log_format(LOG_ERROR, "protocol error, failed to deserialize Configuration Msg.");
		return;
	}

	//Update stm32 modules config from config msg
	end_cmd_set_time(msg.getMatchDuration());
	if( msg.getControlPeriod() )
	{
		periodInTicks = ms_to_tick(msg.getControlPeriod());
	}

	//Start all configured modules
	log_format(LOG_INFO, "Configuration and version request received, started.");
	modules_set_start_config(msg.getStartModuleConfig());
	start_optionnal_modules();

	//Publish ready
	EventMessage evt(EVT_INFORM_READY);
	ArdCom::getInstance().send(evt);

	x86SentConfig = true;
}

void master_sendFeedback()
{
	if(ArdCom::getInstance().isConnected())
	{
		status_msg_data.current_time = systick_get_time();
		status_msg_data.match_time_elapsed = systick_get_match_time().ms;
		status_msg_data.match_time_togo = end_get_match_time_togo();
		status_msg_data.raw_data_gyro = gyro_get_raw_data();
		status_msg_data.omega_gyro = gyro_get_omega();
		status_msg_data.pos_theta_gyro_euler = gyro_get_theta_euler();
		status_msg_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
		status_msg_data.vBat = adc_filtered_data.vBat;
		status_msg_data.iPwm[0] = adc_filtered_data.i[0];
		status_msg_data.iPwm[1] = adc_filtered_data.i[1];
		status_msg_data.iPwm[2] = adc_filtered_data.i[2];
		status_msg_data.iPwm[3] = adc_filtered_data.i[3];
		status_msg_data.pumpState = pumps_get_state();
		status_msg_data.color = (uint8_t) getColor();
		status_msg_data.power_state = power_get();
		dynamixel_update_usb_data(&status_msg_data.dynamixel);

		//Send message on usb com stack
		StatusMessage msg(status_msg_data);
		ArdCom::getInstance().send(msg);
	}
}

void master_command(Datagram& dtg)
{
	X86CmdMsg msg;
	if( !msg.deserialize(dtg.getPayload()) )
	{
		log_format(LOG_ERROR, "protocol error, failed to deserialize x86Cmd Msg.");
		return;
	}

	if( msg.powerOn() )
	{
		power_clear(POWER_OFF);
	}
	else
	{
		power_set(POWER_OFF);
	}

	location_set_position(msg.getPose());

	pump[PUMP_1].set(msg.getPumpCmd(PUMP_1)/100.0f);
	pump[PUMP_2].set(msg.getPumpCmd(PUMP_2)/100.0f);
	pump[PUMP_3].set(msg.getPumpCmd(PUMP_3)/100.0f);
	pump[PUMP_4].set(msg.getPumpCmd(PUMP_4)/100.0f);

	for( int i = 0 ; i < NB_MAX_AX12+NB_MAX_RX24 ; ++i)
	{
		struct dynamixel_cmd_param const * const cmd = msg.getDynamixelCmd(i);
		if( cmd != NULL )
		{
			dynamixel_cmd(cmd);
		}
	}

	heartbeat_kick();

	//Send data to x86
	master_sendFeedback();
}

void master_task(void* arg)
{
	UNUSED(arg);

	portTickType lastWakeTime;

	//On laisse un peu de temps à tout le monde pour booter
	vTaskDelay(ms_to_tick(200));

	lastWakeTime = xTaskGetTickCount ();
	while(1)
	{
		// mise a jour heartbeat
		heartbeat_update();

		//Execution du step de la machine
		master_fsm.execute();

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
