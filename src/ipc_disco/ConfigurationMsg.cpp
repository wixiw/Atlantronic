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

uint8_t ConfigurationMsg::getStartModuleConfig() const
{
	return m_config.start_module_flags;
}

void ConfigurationMsg::setMatchDuration(double durationInSeconds)
{
	m_config.match_duration = durationInSeconds * 1000;
}

void ConfigurationMsg::setModuleStartConfig(BootModuleId id, bool doStart)
{
	//change the bit at position "id" to be equal to doStart
	m_config.start_module_flags ^= (-doStart ^ m_config.start_module_flags) & (1 << id);
}

} /* namespace arp_stm32 */
