/*
 * ArcCom_c_wrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */


#include <map>
#include "kernel/log.h"
#include "ipc_disco/ConfigurationMsg.hpp"
#include "ipc_disco/EventMessage.hpp"
#include "ipc_disco/GyroMsg.hpp"
#include "ipc_disco/LogMessage.hpp"
#include "ipc_disco/VersionMessage.hpp"
#include "ipc_disco/X86CmdMsg.hpp"
#include "boot_signals.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"
#include "ArdCom.h"
#include "kernel/heartbeat.h"
#include "kernel/driver/power.h"
#include "kernel/driver/dynamixel.h"
#include "kernel/driver/gyro.h"
#include "discovery/location.h"
#include "stm32_tasks/control.h"
#include "stm32_tasks/end.h"
#include "stm32_tasks/led.h"
using namespace std;
using namespace arp_stm32;

static char usb_ptask_buffer[400];
static unsigned char x86Connected = 0;

static std::map<EventId, EventCallback>* evtCallbacks;

void registerEventCallback(EventId id, EventCallback fct)
{
	(*evtCallbacks)[id] = fct;
}

unsigned char isX86Connected()
{
	return x86Connected;
}

int deserialize_ard(CircularBuffer const * const buffer)
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
	std::map<EventId, EventCallback>::iterator element = evtCallbacks->find(msg.getEventId());
	if( element == evtCallbacks->end() )
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", dtg.getHeader().type);
	else
		element->second();
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
		dynamixel_cmd(msg.getDynamixelCmd(i));
	}

	heartbeat_kick();
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
	x86Connected = 1; //TODO toujours d'actualite ?
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

	//Register Event callbacks
	evtCallbacks = new std::map<EventId, EventCallback>();
	registerEventCallback(EVT_LIST_TASKS, evtCb_ptaskRequest);
	registerEventCallback(EVT_REBOOT,  evtCb_reboot);
	registerEventCallback(EVT_ENABLE_HEARTBEAT,  heartbeat_enable);
	registerEventCallback(EVT_X86_INIT_DONE, led_inform_x86_ready);
	registerEventCallback(EVT_SCAN_DYNAMIXELS, dynamixel_cmd_scan);
	registerEventCallback(EVT_REQUEST_END_MATCH, end_quit_match);
	com.registerMsgCallback(MSG_EVENT, msgCb_event);

	//Register other messages
	com.registerMsgCallback(MSG_X86_CMD, msgCb_x86Cmd);
	com.registerMsgCallback(MSG_CONFIGURATION, msgCb_configuration);
	com.registerMsgCallback(MSG_GYRO_CMD, msgCb_gyro);
}
