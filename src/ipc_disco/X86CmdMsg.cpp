/*
 * X86CmdMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "X86CmdMsg.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

namespace arp_stm32
{



X86CmdMsg::X86CmdMsg()
    : IpcMsg()
    , m_cmd()
{
}

X86CmdMsg::~X86CmdMsg(){}

bool X86CmdMsg::serialize(Payload& payload) const
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

bool X86CmdMsg::deserialize(PayloadConst payload)
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

MsgType X86CmdMsg::getType() const
{
    return MSG_X86_CMD;
}

MsgSize X86CmdMsg::getSize() const
{
	return SIZE;
}

bool X86CmdMsg::powerOn() const
{
	return m_cmd.power_request != 0;
}

VectPlan const & X86CmdMsg::getPose() const
{
	return m_cmd.position;
}

uint8_t X86CmdMsg::getPumpCmd(PumpId id) const
{
	return m_cmd.pumpCmd[id];
}

struct dynamixel_cmd_param const * X86CmdMsg::getDynamixelCmd(uint8_t id) const
{
	if( 2 <= m_cmd.dynamixel_cmd[id].id && m_cmd.dynamixel_cmd[id].id < 0xfe )
	{
		return &(m_cmd.dynamixel_cmd[id]);
	}
	else
	{
		return NULL;
	}

}

void X86CmdMsg::setPose(VectPlan const & pose)
{
	m_cmd.position = pose;
}

void X86CmdMsg::powerOn(bool requestPowerOn)
{
	if( requestPowerOn )
	{
		m_cmd.power_request = 1;
	}
	else
	{
		m_cmd.power_request = 0;
	}
}

void X86CmdMsg::setPumpCmd(PumpId id, uint8_t cmd)
{
	if( cmd > 100 )
		cmd = 100;

	m_cmd.pumpCmd[id] = cmd;
}

} /* namespace arp_stm32 */
