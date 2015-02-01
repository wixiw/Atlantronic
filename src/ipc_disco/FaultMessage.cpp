/*
 * FaultMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "FaultMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

namespace arp_stm32
{



FaultMessage::FaultMessage()
    : IpcMsg()
{
    memset(m_fault, 0, sizeof(m_fault));
}

FaultMessage::~FaultMessage(){}

bool FaultMessage::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, m_fault, SIZE);
    payload.second = SIZE;
    return true;
}

bool FaultMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(m_fault, payload.first, SIZE);
    return true;
}

MsgType FaultMessage::getType() const
{
    return MSG_FAULT;
}


} /* namespace arp_stm32 */
