/*
 * EventMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "EventMessage.hpp"
#include "DiscoveryIpcTypes.hpp"

namespace arp_stm32
{

using namespace arp_stm32::ipc;

EventMessage::EventMessage()
    : IpcMsg()
    , m_id(EVT_RESERVED)
{
}

EventMessage::EventMessage(ipc::EventId id)
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

EventId const & EventMessage::getEventId() const
{
    return m_id;
}

MsgType EventMessage::getType() const
{
    return MSG_EVENT;
}

EventMessage::~EventMessage(){}

} /* namespace arp_stm32 */
