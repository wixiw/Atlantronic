/*
 * EventMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "EventMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

namespace arp_stm32
{


EventMessage::EventMessage()
    : IpcMsg()
    , m_id(EVT_RESERVED)
{
}

EventMessage::EventMessage(EventId id)
    : IpcMsg()
    , m_id(id)
{
}

bool EventMessage::serialize(Payload& payload) const
{
    memcpy(payload.first, &m_id, SIZE);
    payload.second = SIZE;
    return true;
}

bool EventMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(&m_id, payload.first, SIZE);
    return true;
}

EventId EventMessage::getEventId() const
{
    return static_cast<EventId>(m_id);
}

MsgType EventMessage::getType() const
{
    return MSG_EVENT;
}

MsgSize EventMessage::getSize() const
{
	return SIZE;
}

EventMessage::~EventMessage(){}

} /* namespace arp_stm32 */
