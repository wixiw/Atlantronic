/*
 * LogMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#ifndef LOGMESSAGE_HPP_
#define LOGMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "kernel/log_level.h"
#include <string>

namespace arp_stm32
{

class LogMessage: public arp_stm32::IpcMsg
{
    public:
        LogMessage();
        virtual ~LogMessage();

        static const MsgSize MAX_FUNCTION_LENGTH = 20;
        static const MsgSize MAX_LOG_SIZE = MSG_MAX_SIZE - MAX_FUNCTION_LENGTH - sizeof(uint16_t) - sizeof(log_level);
        static const MsgSize SIZE = MAX_LOG_SIZE + MAX_FUNCTION_LENGTH + sizeof(uint16_t) + sizeof(log_level);

        /**
         * Overloaded \see IpcMsg
         */
        virtual bool serialize(Payload& payload) const;

        /**
         * Overloaded \see IpcMsg
         */
        virtual bool deserialize(PayloadConst payload);

        /**
         * Overloaded \see IpcMsg
         */
        virtual MsgType getType() const;

        void log(enum log_level lvl, std::string function, uint16_t line, std::string log);

    protected:
        enum log_level m_level;
        uint16_t m_line;
        char m_function[MAX_FUNCTION_LENGTH];
        char m_log[MAX_LOG_SIZE];
};

} /* namespace arp_stm32 */
#endif /* LOGMESSAGE_HPP_ */
