/*
 * RawMessage.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef RAWMESSAGE_HPP_
#define RAWMESSAGE_HPP_

#include "ipc/IpcMsg.hpp"

namespace arp_stm32
{

/**
 * This class should rarely be used. It's a default implementation of an UsbMessage
 */
class RawMessage: public IpcMsg
{
    public:
        RawMessage();
        virtual ~RawMessage(){};

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
         * Return the body pointer
         */
        uint8_t const * getPayload() const;

        /**
         * Define the payload. It will write zeroes at the end to fill m_data
         * @param data : a pointer to the payload to copy, if NULL will fill m_data with zeroes
         * @param size : the size of the payload. You have to ensure that size is less than UsbMessage::MSG_MAX_SIZE
         */
        void setPayload(uint8_t const * const data, MsgSize size);

    protected:
        uint8_t m_data[MSG_MAX_SIZE];
        MsgSize m_payloadSize;


};

} /* namespace arp_stm32 */
#endif /* RAWMESSAGE_HPP_ */
