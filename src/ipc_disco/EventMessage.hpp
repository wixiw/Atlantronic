/*
 * EventMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef EVENTMESSAGE_HPP_
#define EVENTMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "DiscoveryIpcTypes.h"

namespace arp_stm32
{

class EventMessage: public arp_stm32::IpcMsg
{
    public:
        static const MsgSize SIZE = sizeof(uint8_t);

        EventMessage();
        EventMessage(EventId id);
        virtual ~EventMessage();


        EventId getEventId() const;


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
        uint8_t m_id;
};

} /* namespace arp_stm32 */
#endif /* EVENTMESSAGE_HPP_ */
