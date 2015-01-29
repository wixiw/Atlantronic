/*
 * EventMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef EVENTMESSAGE_HPP_
#define EVENTMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "DiscoveryIpcTypes.hpp"

namespace arp_stm32
{

class EventMessage: public arp_stm32::IpcMsg
{
    public:
        static const ipc::MsgSize SIZE = sizeof(ipc::EventId);

        EventMessage();
        EventMessage(ipc::EventId id);
        virtual ~EventMessage();


        ipc::EventId const & getEventId() const;


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
        ipc::EventId m_id;
};

} /* namespace arp_stm32 */
#endif /* EVENTMESSAGE_HPP_ */
