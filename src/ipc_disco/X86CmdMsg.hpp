/*
 * X86CmdMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef X86CmdMsg_HPP_
#define X86CmdMsg_HPP_

#include "ipc/IpcMsg.hpp"
#include "discovery/x86_cmd.h"

namespace arp_stm32
{

class X86CmdMsg: public arp_stm32::IpcMsg
{
    public:
        X86CmdMsg();
        virtual ~X86CmdMsg();

        static const MsgSize SIZE = sizeof(x86_cmd);

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
         * accessor on power. The message has to be deserialized.
         * return true if message contains a power on request, false otherwise
         */
        bool powerOn() const;

        /**
         * accessor on position. The message has to be deserialized.
         */
        VectPlan const & getPose() const;

        /**
         * accessor
         */
        uint8_t getPumpCmd(PumpId id) const;

        /**
         * accessor
         * May return NULL if the id is not un the correct range
         */
        struct dynamixel_cmd_param const * getDynamixelCmd(uint8_t id) const;

        /**
         * mutator
         */
        void setPose(VectPlan const & pose);

        /**
         * mutator
         */
        void powerOn(bool requestPowerOn);

        /**
         * mutator
         * cmd in %
         */
        void setPumpCmd(PumpId id, uint8_t cmd);

    protected:
        x86_cmd m_cmd;
};

} /* namespace arp_stm32 */
#endif /* X86CmdMsg_HPP_ */
