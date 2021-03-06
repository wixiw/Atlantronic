/*
 * StatusMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#include "StatusMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include "components/robot/power.h"
#include "core/gpio.h"
#include "components/robot/color.h"

namespace arp_stm32
{



StatusMessage::StatusMessage()
    : IpcMsg()
{
    memset(&m_data, 0, sizeof(control_usb_data));
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

bool StatusMessage::isPumpBlocked(uint8_t id) const
{
    return (m_data.pumpState >> id) & 0x01;
}

double StatusMessage::getMatchTimeElapsed() const
{
    return m_data.match_time_elapsed/1000.;
}

double StatusMessage::getMatchTimeRemaining() const
{
    return m_data.match_time_togo/1000.;
}


char const * StatusMessage::getColor() const
{
    switch (m_data.color)
    {
        case COLOR_SYM:
            return "green";
            break;
        case COLOR_PREF:
            return "yellow";
            break;
        default:
            return "unknown";
            break;
    }
}

} /* namespace arp_stm32 */
