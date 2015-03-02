/*
 * ArcCom_c_wrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */

#define WEAK_USB

#include "core/gpio.h"

#include "com/msgs/DiscoveryIpcMessage.hpp"
#include "com/stack_com/ArdCom.hpp"
#include "com/stack_com/heartbeat.h"

#include "components/gyro/gyro.h"
#include "components/log/log.h"

#include "os/os.h"
#include "priority.h"

#include "ArdCom_c_wrapper.h"
#include "end.h"

using namespace arp_stm32;

static char usb_ptask_buffer[400];

static EventCallback evtCallbacks[EVT_NB];

void registerEventCallback(EventId id, EventCallback fct)
{
	evtCallbacks[id] = fct;
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

	//Register Event callbacks
	registerEventCallback(EVT_LIST_TASKS, evtCb_ptaskRequest);
	registerEventCallback(EVT_REBOOT,  evtCb_reboot);
	registerEventCallback(EVT_SCAN_DYNAMIXELS, dynamixel_cmd_scan);
	registerEventCallback(EVT_REQUEST_END_MATCH, end_quit_match);
	registerEventCallback(EVT_X86_READY_FOR_MATCH, setX86ReadyForMatch);
	com.registerMsgCallback(MSG_EVENT, msgCb_event);

	//Register other messages
	com.registerMsgCallback(MSG_X86_CMD, master_command);
	com.registerMsgCallback(MSG_CONFIGURATION, master_configuration);
	com.registerMsgCallback(MSG_GYRO_CMD, msgCb_gyro);
}
