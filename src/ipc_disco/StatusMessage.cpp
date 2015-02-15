/*
 * StatusMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#include "StatusMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include "kernel/driver/power.h"
#include <cstring>

namespace arp_stm32
{



StatusMessage::StatusMessage()
    : IpcMsg()
    , m_data()
{
}

StatusMessage::StatusMessage(control_usb_data const& data)
: IpcMsg()
, m_data(data)
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

MsgSize StatusMessage::getSize() const
{
	return SIZE;
}

double StatusMessage::getTime() const
{
    return m_data.current_time.ms*1E-3 + m_data.current_time.ns*1E-9;
}

bool StatusMessage::isPowerOn() const
{
    return getRawPowerData() == POWER_ON;
}

bool StatusMessage::isEmergencyStopActive() const
{
    return getRawPowerData() & POWER_OFF_AU;
}

bool StatusMessage::isUnderVoltageErrorActive() const
{
    return getRawPowerData() & POWER_OFF_UNDERVOLTAGE;
}

bool StatusMessage::isPowerAllowedByStragety() const
{
    return getRawPowerData() & POWER_OFF;
}

bool StatusMessage::isPowerShutdownAtEndOfMatch() const
{
    return getRawPowerData() & POWER_OFF_END_MATCH;
}

bool StatusMessage::isHeartBeatLost() const
{
    return getRawPowerData() & POWER_OFF_HEARTBEAT;
}

int StatusMessage::getRawPowerData() const
{
    return m_data.power_state;
}

double StatusMessage::getBatteryVoltage() const
{
    return m_data.vBat;
}

int StatusMessage::getRawGpioData() const
{
    return m_data.gpio;
}


} /* namespace arp_stm32 */
