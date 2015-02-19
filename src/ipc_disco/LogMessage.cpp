/*
 * LogMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#include "LogMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstdio>

namespace arp_stm32
{


using namespace std;

LogMessage::LogMessage()
    : IpcMsg()
	, m_level(LOG_ERROR)
    , m_log()
{
    strcpy(reinterpret_cast<char*>(m_log), "EMPTY !");
}

bool LogMessage::serialize(Payload& payload) const
{
    if( MAX_LOG_SIZE < getSize() )
    {
        payload.second = 0;
        return false;
    }


    uint8_t * currentPos = payload.first;
    memcpy(currentPos, &m_level, sizeof(m_level));
    currentPos += sizeof(m_level);
    
    size_t size = strlen(reinterpret_cast<char const*>(m_log))+1;
    memcpy(currentPos, &m_log, size);

    
    payload.second = getSize();
    return true;
}

bool LogMessage::deserialize(PayloadConst payload)
{
    if( MAX_LOG_SIZE < payload.second  )
    {
        return false;
    }

    uint8_t const * currentPos = payload.first;
    memcpy(&m_level, currentPos, sizeof(m_level));
    currentPos += sizeof(m_level);
    
    memcpy(&m_log, currentPos, payload.second - sizeof(m_level));
    

    return true;
}

void LogMessage::logArd(enum log_level level, char const * const func, uint16_t line, char const * const log_)
{
	m_level =static_cast<uint8_t>(level);
	snprintf(reinterpret_cast<char*>(m_log), MAX_LOG_SIZE, "%s():%d : %s", func, line, log_);
}

MsgType LogMessage::getType() const
{
    return MSG_LOG;
}

MsgSize LogMessage::getSize() const
{
	return strlen(reinterpret_cast<const char*>(m_log))+1+sizeof(m_level);
}

char const * LogMessage::getLogText() const
{
	return reinterpret_cast<const char*>(m_log);
}

log_level LogMessage::getLevel() const
{
    return static_cast<log_level>(m_level);
}

char const * LogMessage::getLevelText() const
{
	switch(static_cast<log_level>(m_level))
	{
		case LOG_ERROR:
			return "ERROR";
			break;
		case LOG_INFO:
			return "INFO";
			break;
		case LOG_DEBUG1:
		case LOG_DEBUG2:
		case LOG_DEBUG3:
			return "DEBUG";
			break;
		case LOG_MAX:
		default:
			return "???";
			break;
	}
	return "";
}

LogMessage::~LogMessage(){}

} /* namespace arp_stm32 */
