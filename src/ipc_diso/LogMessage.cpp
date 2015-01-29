/*
 * LogMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#include "LogMessage.hpp"
#include "DiscoveryIpcTypes.hpp"
#include <sstream>

namespace arp_stm32
{

using namespace arp_stm32::ipc;
using namespace std;

LogMessage::LogMessage()
    : IpcMsg()
    , m_level(LOG_MAX)
    , m_line(0)
    , m_function()
    , m_log()
{
    strcpy(m_function, "unknown");
    strcpy(m_log, "EMPTY !");
}

bool LogMessage::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    uint8_t * currentPos = payload.first;
    memcpy(currentPos, &m_level, sizeof(m_level));
    currentPos += sizeof(m_level);

    memcpy(currentPos, &m_function, MAX_FUNCTION_LENGTH);
    currentPos += MAX_FUNCTION_LENGTH;

    memcpy(currentPos, &m_line, sizeof(m_line));
    currentPos += sizeof(m_line);

    memcpy(currentPos, &m_log, MAX_LOG_SIZE);
    currentPos += MAX_LOG_SIZE;

    payload.second = currentPos - payload.first;
    return true;
}

bool LogMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE )
    {
        return false;
    }

    uint8_t const * currentPos = payload.first;
    memcpy(&m_level, currentPos, sizeof(m_level));
    currentPos += sizeof(m_level);

    memcpy(&m_function, currentPos, MAX_FUNCTION_LENGTH);
    currentPos += MAX_FUNCTION_LENGTH;

    memcpy(&m_line, currentPos, sizeof(m_line));
    currentPos += sizeof(m_line);

    memcpy(&m_log, currentPos, MAX_LOG_SIZE);
    currentPos += MAX_LOG_SIZE;

    return true;
}

string LogMessage::getLogString() const
{
    ostringstream os;
    os << "[" << m_level << "] " << string(m_function) << "() at l=" << m_line << " : " << string(m_log);
    return os.str();
}

void LogMessage::log(enum log_level lvl, std::string function, uint16_t line, std::string log)
{
    m_level = lvl;
    strcpy(m_function, function.c_str());
    m_line = line;
    strcpy(m_log, log.c_str());
}

MsgType LogMessage::getType() const
{
    return MSG_LOG;
}

LogMessage::~LogMessage(){}

} /* namespace arp_stm32 */
