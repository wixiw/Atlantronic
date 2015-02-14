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

namespace arp_stm32
{

class LogMessage: public arp_stm32::IpcMsg
{
    public:
        LogMessage();
        virtual ~LogMessage();

        static const MsgSize MAX_LOG_SIZE = MSG_MAX_SIZE - 1 - sizeof(log_level);

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

        /**
         * Overloaded \see IpcMsg
         */
        virtual MsgSize getSize() const;

        char const * getLogText() const;
        log_level getLevel() const;
        char const * getLevelText() const;

        void logArd(enum log_level lvl, char const * const function_, uint16_t line, char const * const log_);

    protected:
	    uint8_t m_level;
	    uint8_t m_log[MAX_LOG_SIZE];
};

} /* namespace arp_stm32 */
#endif /* LOGMESSAGE_HPP_ */
