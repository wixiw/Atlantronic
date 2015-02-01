/*
 * StatusMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#include "StatusMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

namespace arp_stm32
{



StatusMessage::StatusMessage()
    : IpcMsg()
    , m_data()
{
}

StatusMessage::~StatusMessage(){}

bool StatusMessage::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, &m_data, SIZE);
    payload.second = SIZE;
    return true;
}

bool StatusMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(&m_data, payload.first, SIZE);
    return true;
}

MsgType StatusMessage::getType() const
{
    return MSG_STATUS;
}

} /* namespace arp_stm32 */
