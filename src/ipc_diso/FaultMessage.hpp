/*
 * FaultMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef FAULTMESSAGE_HPP_
#define FAULTMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "kernel/fault.h"

namespace arp_stm32
{

class FaultMessage : IpcMsg
{
    public:
        FaultMessage();
        virtual ~FaultMessage();

        static const ipc::MsgSize SIZE = sizeof(fault_status)*FAULT_MAX;

        /**
         * Overloaded \see IpcMsg
         */
        virtual bool serialize(ipc::Payload& payload) const;

        /**
         * Overloaded \see IpcMsg
         */
        virtual bool deserialize(ipc::PayloadConst payload);

        /**
         * Overloaded \see IpcMsg
         */
        virtual ipc::MsgType getType() const;

    protected:
        struct fault_status m_fault[FAULT_MAX];
};

} /* namespace arp_stm32 */
#endif /* FAULTMESSAGE_HPP_ */
