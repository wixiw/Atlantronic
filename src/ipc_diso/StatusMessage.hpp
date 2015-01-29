/*
 * StatusMessage.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef STATUSMESSAGE_HPP_
#define STATUSMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"
#include "linux/tools/robot_interface.h"

namespace arp_stm32
{

class StatusMessage: public arp_stm32::IpcMsg
{
    public:
        StatusMessage();
        virtual ~StatusMessage();

        static const ipc::MsgSize SIZE = sizeof(control_usb_data);

        /**
         * Overloaded \see IpcMsg
         * convert the string version
         */
        virtual bool serialize(ipc::Payload& payload) const;

        /**
         * Overloaded \see IpcMsg
         * ensure size is 41
         */
        virtual bool deserialize(ipc::PayloadConst payload);

        /**
         * Overloaded \see IpcMsg
         */
        virtual ipc::MsgType getType() const;

    protected:
        control_usb_data m_data;
};

} /* namespace arp_stm32 */
#endif /* STATUSMESSAGE_HPP_ */
