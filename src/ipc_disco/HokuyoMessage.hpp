/*
 * HokuyoMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef HokuyoMessage_HPP_
#define HokuyoMessage_HPP_

#include "ipc/IpcMsg.hpp"
#include "kernel/driver/hokuyo.h"

namespace arp_stm32
{

class HokuyoMessage: public arp_stm32::IpcMsg
{
    public:
        HokuyoMessage();
        HokuyoMessage(struct hokuyo_scan const& scan);
        virtual ~HokuyoMessage();

        static const MsgSize SIZE = sizeof(hokuyo_scan);

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


    protected:
        struct hokuyo_scan m_scan;
};

} /* namespace arp_stm32 */
#endif /* HokuyoMessage_HPP_ */
