/*
 * StatusMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef STATUSMESSAGE_HPP_
#define STATUSMESSAGE_HPP_

#include "com/stack_com/IpcMsg.hpp"
#include "master/master.h"

namespace arp_stm32
{

class StatusMessage: public arp_stm32::IpcMsg
{
    public:
        StatusMessage();
        StatusMessage(control_usb_data const&  data);
        virtual ~StatusMessage();

        static const MsgSize SIZE = sizeof(control_usb_data);

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

        //time in seconds since boot
        double getTime() const;
        bool isPowerOn() const;
        bool isEmergencyStopActive() const;
        bool isUnderVoltageErrorActive() const;
        bool isPowerAllowedByStragety() const;
        bool isPowerShutdownAtEndOfMatch() const;
        bool isHeartBeatLost() const;
        int getRawPowerData() const;
        //in volts
        double getBatteryVoltage() const;
        int getRawGpioData() const;
        bool getGpio(uint32_t mask) const;
        bool isPumpBlocked(uint8_t id) const;
        double getMatchTimeElapsed() const;
        double getMatchTimeRemaining() const;
        char const * getColor() const;

    protected:
        control_usb_data m_data;
};

} /* namespace arp_stm32 */
#endif /* STATUSMESSAGE_HPP_ */
