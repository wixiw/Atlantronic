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

namespace arp_stm32
{

class OpponentListMsg: public arp_stm32::IpcMsg
{
    public:
		OpponentListMsg();
        OpponentListMsg(detection_object const * const objs, uint8_t const obj_size);
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

        detection_object const * getOppList() const;

        /**
         * Returns false if the list is full.
         */
        bool addOpponent(detection_object& opp);

    protected:
        uint8_t m_oppListSize;
        detection_object m_oppList[DETECTION_NUM_OBJECT];
};

} /* namespace arp_stm32 */
#endif /* OPPONENTLISTMSG_HPP_ */
