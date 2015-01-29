/*
 * OpponentListMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef OPPONENTLISTMSG_HPP_
#define OPPONENTLISTMSG_HPP_

#include "ipc/IpcMsg.hpp"
#include "discovery/detection.h"
#include <list>
#include <string>

namespace arp_stm32
{

class OpponentListMsg: public arp_stm32::IpcMsg
{
    public:
        OpponentListMsg();
        virtual ~OpponentListMsg();

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

        /**
         * For debug purposes
         */
        virtual std::string toString() const;

        std::list<detection_object> const & getOppList() const;

        void addOpponent(detection_object& opp);

    protected:
        std::list<detection_object> m_oppList;
};

} /* namespace arp_stm32 */
#endif /* OPPONENTLISTMSG_HPP_ */
