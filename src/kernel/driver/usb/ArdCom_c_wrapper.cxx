/*
 * ArcCom_c_wrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: willy
 */


#include <map>
#include "kernel/log.h"
#include "ipc_disco/EventMessage.hpp"
#include "ipc_disco/LogMessage.hpp"
#include "ipc_disco/VersionMessage.hpp"
#include "boot_signals.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"
#include "ArdCom.h"

using namespace std;
using namespace arp_stm32;

static char usb_ptask_buffer[400];
static unsigned char usb_get_version_done = 0;

static std::map<EventId, EventCallback>* evtCallbacks;

void registerEventCallback(EventId id, EventCallback fct)
{
	(*evtCallbacks)[id] = fct;
}

unsigned char usb_is_get_version_done()
{
	return usb_get_version_done;
}

int deserialize_ard(CircularBuffer const * const buffer)
{
	return ArdCom::getInstance().deserialize(buffer);
}

void evtCb_ptaskRequest()
{
	vTaskGetRunTimeStats(usb_ptask_buffer, sizeof(usb_ptask_buffer));
	log(LOG_INFO, usb_ptask_buffer);
}

void evtCb_versionRequest()
{
	usb_get_version_done = 1;
	VersionMessage msg;

	ArdCom::getInstance().send(msg);

	int i;
	for( i = 0 ; i < BOOT_SIGNAL_NB ; i++ )
	{
		boot_signals[i]->set();
	}

	log_format(LOG_INFO, "Version request received, started.");
}

void evtCb_reboot()
{
	reboot();
}

void usb_add_log(enum log_level level, const char* func, uint16_t line, const char* log_)
{
	LogMessage msg;
	msg.logArd(level, func, line, log_);
	ArdCom::getInstance().send(msg);
}

void msgCb_event(Datagram& dtg)
{
	EventMessage msg;
	if( !msg.deserialize(dtg.getPayload()) )
	{
		log_format(LOG_ERROR, "protocol error, failed to deserialize event.");
		return;
	}

	//Call event callback associated to received event id.
	std::map<EventId, EventCallback>::iterator element = evtCallbacks->find(msg.getEventId());
	if( element == evtCallbacks->end() )
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", dtg.getHeader().type);
	else
		element->second();
}



void usb_ard_init()
{
	ArdCom::getInstance().init();
	evtCallbacks = new std::map<EventId, EventCallback>();
	registerEventCallback(EVT_REQUEST_VERSION, evtCb_versionRequest);
	registerEventCallback(EVT_LIST_TASKS, evtCb_ptaskRequest);
	registerEventCallback(EVT_REBOOT,  evtCb_reboot);
	ArdCom::getInstance().registerMsgCallback(MSG_EVENT, msgCb_event);
}
