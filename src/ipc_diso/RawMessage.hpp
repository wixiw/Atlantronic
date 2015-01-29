/*
 * RawMessage.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef RAWMESSAGE_HPP_
#define RAWMESSAGE_HPP_

#include "IpcMsg.hpp"
#include <string>

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
         * convert the string verison
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

        /**
         * Return the total length of the message body (current size)
         */
        ipc::MsgSize getPayloadSize() const;

        /**
         * Return the body pointer
         */
        uint8_t const * getPayload() const;

        /**
         * Define the payload. It will write zeroes at the end to fill m_data
         * @param data : a pointer to the payload to copy, if NULL will fill m_data with zeroes
         * @param size : the size of the payload. You have to ensure that size is less than UsbMessage::MSG_MAX_SIZE
         */
        void setPayload(uint8_t const * const data, ipc::MsgSize size);


        /**
         * Overloaded \see UsbMessage
         */
        virtual std::string toString() const;

    protected:
        uint8_t m_data[ipc::MSG_MAX_SIZE];
        ipc::MsgSize m_payloadSize;


};

} /* namespace arp_stm32 */
#endif /* RAWMESSAGE_HPP_ */
