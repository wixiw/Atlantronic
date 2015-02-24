/*
 * GyroMsg.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef GyroMsg_HPP_
#define GyroMsg_HPP_

#include "com/stack_com/IpcMsg.hpp"
#include "components/gyro/gyro.h"

namespace arp_stm32
{

class GyroMsg: public arp_stm32::IpcMsg
{
    public:
        GyroMsg();
        GyroMsg(gyro_cmd const & cmd);
        virtual ~GyroMsg();

        static const MsgSize SIZE = sizeof(gyro_cmd);

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

        /**
         * accessor. The message has to be deserialized.
         */
        gyro_cmd const & getCmd() const;


    protected:
        gyro_cmd m_cmd;
};

} /* namespace arp_stm32 */
#endif /* GyroMsg_HPP_ */
