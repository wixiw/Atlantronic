/*
 * GyroMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "GyroMsg.hpp"
#include "DiscoveryIpcTypes.h"

namespace arp_stm32
{

GyroMsg::GyroMsg()
    : IpcMsg()
    , m_cmd()
{
}


GyroMsg::GyroMsg(gyro_cmd const & cmd)
    : IpcMsg()
    , m_cmd(cmd)
{
}

GyroMsg::~GyroMsg(){}

bool GyroMsg::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, &m_cmd, SIZE);
    payload.second = SIZE;
    return true;
}

bool GyroMsg::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(&m_cmd, payload.first, SIZE);
    return true;
}

MsgType GyroMsg::getType() const
{
    return MSG_GYRO_CMD;
}

MsgSize GyroMsg::getSize() const
{
	return SIZE;
}

gyro_cmd const & GyroMsg::getCmd() const
{
	return m_cmd;
}


} /* namespace arp_stm32 */
