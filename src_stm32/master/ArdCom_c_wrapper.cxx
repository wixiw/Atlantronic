/*
 * ArcCom_c_wrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */

#define WEAK_USB

#include "os/module.h"
#include "core/gpio.h"
#include "core/boot_signals.h"

#include "com/msgs/DiscoveryIpcMessage.hpp"
#include "com/stack_com/ArdCom.hpp"
#include "com/stack_com/heartbeat.h"

#include "components/robot/power.h"
#include "components/dynamixel/dynamixel.h"
#include "components/gyro/gyro.h"
#include "components/localization/location.h"
#include "components/led/led.h"
#include "components/log/log.h"

#include "os/os.h"
#include "priority.h"

#include "ArdCom_c_wrapper.h"
#include "control.h"
#include "end.h"

using namespace arp_stm32;

static char usb_ptask_buffer[400];
static bool x86Connected = false;
static bool x86ReadyForMatch = false;

static EventCallback evtCallbacks[EVT_NB];

void registerEventCallback(EventId id, EventCallback fct)
{
	evtCallbacks[id] = fct;
}

//TODO a deplacer dans master
bool isX86Connected()
{
	return x86Connected;
}

bool isX86ReadyForMatch()
{
	return x86ReadyForMatch;
}

void setX86ReadyForMatch()
{
	x86ReadyForMatch = true;
}

int usb_received_buffer_CB(CircularBuffer * const buffer)
{
	return ArdCom::getInstance().deserialize(buffer);
}

void sendBootup()
{
	//Provide version
	VersionMessage msgV;
	ArdCom::getInstance().send(msgV);
}

void evtCb_ptaskRequest()
{
	vTaskGetRunTimeStats(usb_ptask_buffer, sizeof(usb_ptask_buffer));
	log(LOG_INFO, usb_ptask_buffer);
}

void evtCb_reboot()
{
	reboot();
}

void msgCb_event(Datagram& dtg)
{
	EventMessage msg;
	if( !msg.deserialize(dtg.getPayload()) )
	{
		log_format(LOG_ERROR, "protocol error, failed to deserialize event Msg.");
		return;
	}

	//Call event callback associated to received event id.
	if(msg.getEventId() < EVT_NB && evtCallbacks[msg.getEventId()] != NULL )
	{
		evtCallbacks[msg.getEventId()]();
	}
	else
	{
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", dtg.getHeader().type);
	}
}

void msgCb_x86Cmd(Datagram& dtg)
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

	//log_format(LOG_INFO, "X86 msg.");
}


void msgCb_configuration(Datagram& dtg)
{
	if( x86Connected )
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
	set_control_period(msg.getControlPeriod());


	//Start all configured modules
	log_format(LOG_INFO, "Configuration and version request received, started.");
	modules_set_start_config(msg.getStartModuleConfig());
	start_all_modules();

	//Publish ready
	x86Connected = 1;
	EventMessage evt(EVT_INFORM_READY);
	ArdCom::getInstance().send(evt);
}

void msgCb_gyro(Datagram& dtg)
{
	GyroMsg msg;
	if( !msg.deserialize(dtg.getPayload()) )
	{
		log_format(LOG_ERROR, "protocol error, failed to deserialize Gyro Msg.");
		return;
	}

	gyro_calibration_cmd(msg.getCmd());
}

void usb_add_log(enum log_level level, const char* func, uint16_t line, const char* log_)
{
	LogMessage msg;
	msg.logArd(level, func, line, log_);
	ArdCom::getInstance().send(msg);
}

void usb_ard_init()
{
	ArdCom& com = ArdCom::getInstance();

	com.init();

	for( int i = 0 ; i < EVT_NB ; i++ )
	{
		evtCallbacks[i] = NULL;
	}

	//TODO verifier connection avec master...

	//Register Event callbacks
	registerEventCallback(EVT_LIST_TASKS, evtCb_ptaskRequest);
	registerEventCallback(EVT_REBOOT,  evtCb_reboot);
	registerEventCallback(EVT_ENABLE_HEARTBEAT,  heartbeat_enable);
	registerEventCallback(EVT_SCAN_DYNAMIXELS, dynamixel_cmd_scan);
	registerEventCallback(EVT_REQUEST_END_MATCH, end_quit_match);
	registerEventCallback(EVT_X86_READY_FOR_MATCH, setX86ReadyForMatch);
	com.registerMsgCallback(MSG_EVENT, msgCb_event);

	//Register other messages
	com.registerMsgCallback(MSG_X86_CMD, msgCb_x86Cmd);
	com.registerMsgCallback(MSG_CONFIGURATION, msgCb_configuration);
	com.registerMsgCallback(MSG_GYRO_CMD, msgCb_gyro);
}
