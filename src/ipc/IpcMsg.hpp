/*
 * IpcMsg.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef IPCMSG_HPP_
#define IPCMSG_HPP_

#include "IpcTypes.hpp"
#include <cstring>

namespace arp_stm32
{

class Datagram;
class IpcMsg
{
    public:
        IpcMsg();
        virtual ~IpcMsg();

        /**
         * Overloaded \see IpcMsg
         * convert the string verison
         */
        virtual bool serialize(ipc::Payload& payload) const = 0;

        /**
         * Overloaded \see IpcMsg
         * ensure size is 41
         */
        virtual bool deserialize(ipc::PayloadConst payload) = 0;

        /**
         * Get associated ID for header construction
         */
        virtual ipc::MsgType getType() const = 0;

        bool fillDatagram(Datagram& dtg) const;
};

} /* namespace arp_stm32 */
#endif /* IPCMSG_HPP_ */
