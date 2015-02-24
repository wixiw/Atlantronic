/*
 * OpponentListMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef OPPONENTLISTMSG_HPP_
#define OPPONENTLISTMSG_HPP_

#include "com/stack_com/IpcMsg.hpp"
#include "components/localization/detection.h"
#include <list>

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

        std::list<detection_object> const & getOppList() const;

        void addOpponent(detection_object& opp);

    protected:
        std::list<detection_object> m_oppList;
};

} /* namespace arp_stm32 */
#endif /* OPPONENTLISTMSG_HPP_ */
