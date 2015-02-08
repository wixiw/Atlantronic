/*
 * FaultMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef FAULTMESSAGE_HPP_
#define FAULTMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "stm32_tasks/fault.h"

namespace arp_stm32
{

class FaultMessage : public IpcMsg
{
    public:
        FaultMessage();
        FaultMessage(struct fault_status const * const fault);
        virtual ~FaultMessage();

        static const MsgSize SIZE = sizeof(fault_status)*FAULT_MAX;

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

    protected:
        struct fault_status m_fault[FAULT_MAX];
};

} /* namespace arp_stm32 */
#endif /* FAULTMESSAGE_HPP_ */
