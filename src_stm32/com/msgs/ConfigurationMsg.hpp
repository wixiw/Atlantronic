/*
 * ConfigurationMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef ConfigurationMsg_HPP_
#define ConfigurationMsg_HPP_

#include "com/stack_com/IpcMsg.hpp"
#include "com/stm32_config.h"

namespace arp_stm32
{

class ConfigurationMsg: public arp_stm32::IpcMsg
{
    public:
        ConfigurationMsg();
        virtual ~ConfigurationMsg();

        static const MsgSize SIZE = sizeof(stm32_config);

        /**
         * Overloaded \see IpcMsg
         */
        virtual bool serialize(Payload& payload) const;

        /**
         * Overloaded \see IpcMsg
         * ensure size is 41
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

        /**
         * Accessor
         * time of match in ms
         */
        uint32_t getMatchDuration() const;

        /**
         * Accessor
         */
        bool isHokuyoDebug() const;

        /**
         * Accessor
         */
        uint8_t getControlPeriod() const;

        /**
         * Accessor
         */
        uint8_t getHeartbeatTimeout() const;

        /**
         * Configure the match duration
         */
        void setMatchDuration(double durationInSeconds);

        /**
         * Call this to let hokuyo send their raw scan on USB
         */
        void setHokuyoDebug();

        /**
         * Configure the control task period, in seconds.
         */
        void setControlPeriod(double periodInS);

        /**
         * Configure the timeout of the communication watchdog
         * A configuration of 0 mean no hearbeat detection
         */
        void setHeartbeatTimeout(double timeoutInS);


    protected:
        static const uint8_t HOKUYO_DEBUG =  0X01;

        stm32_config m_config;
};

} /* namespace arp_stm32 */
#endif /* ConfigurationMsg_HPP_ */
