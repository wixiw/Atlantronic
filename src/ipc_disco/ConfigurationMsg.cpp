/*
 * ConfigurationMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "ConfigurationMsg.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

namespace arp_stm32
{

ConfigurationMsg::ConfigurationMsg()
    : IpcMsg()
    , m_config()
{
}

ConfigurationMsg::ConfigurationMsg(stm32_config const& config)
    : IpcMsg()
    , m_config(config)
{
}

ConfigurationMsg::~ConfigurationMsg(){}

bool ConfigurationMsg::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, &m_config, SIZE);
    payload.second = SIZE;
    return true;
}

bool ConfigurationMsg::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(&m_config, payload.first, SIZE);
    return true;
}

MsgType ConfigurationMsg::getType() const
{
    return MSG_CONFIGURATION;
}

MsgSize ConfigurationMsg::getSize() const
{
	return SIZE;
}

uint32_t ConfigurationMsg::getMatchDuration() const
{
	return m_config.match_duration;
}

} /* namespace arp_stm32 */
