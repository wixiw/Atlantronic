/*
 * Datagram.hpp
 *
 *  Created on: Jan 3, 2015
 *      Author: willy
 */

#ifndef DATAGRAM_HPP_
#define DATAGRAM_HPP_

#include "IpcHeader.hpp"

namespace arp_stm32
{

/**
 * A datagram contains all the data requiered to do a deserialization process and get a concrete message.
 * This class is interesting to decouple the UsbController for concrete messages.
 */
class Datagram
{
    public:
        Datagram();
        virtual ~Datagram();

        IpcHeader const& getHeader() const;

        /** It will automatically reset the payload */
        void setHeader(IpcHeader& h);

        /**
         * @return : a read only pointer to the message payload and its dedicated header size
         */
        PayloadConst getPayload() const;

        /**
         * @return : a pointer to the payload in order to fill it.
         */
        Payload getWritablePayload();

        /** Check that the payload has a size matching the header*/
        bool isPayloadFullyReceived() const;

        /**
         * Append some bytes to the payload. This function is not doing the appending job on memory, it is just managing pointers so that
         * an external client can use them safely.
         * You probably want to use this when receiving a message. The datagram is not deserialized yet and you use this function
         * in case you received an incomplete datagram and keep track of the part that have already been received (thus appended to the payload).
         * @return : the buffer position for next append and the remaining size. Returns null in case the payload has the correct size.
         */
        Payload appendPayload(MsgSize size);

        /**
         * Extract some bytes from the payload. This function is not doing the extracting job on memory, it is just managing pointers so that
         * an external client can use them safely.
         * You probably want to use this when sending a msg. The datagram is already serialized and you use this function
         * to do some partial send and keep track of the part that have already been sent (hence extracted from the payload to the usb driver).
         * @return : the buffer position for next extract and the remaining size. Returns null in case the payload has been fully extracted.
         */
        Payload extractPayload(MsgSize size);

        /**
         * Decorator, see IpcHeader.deserialize
         */
        bool deserializeHeader(uint8_t const * const buffer);

        /**
         * Decorator, see IpcHeader.serialize
         */
        bool serializeHeader(uint8_t * const buffer);

        /**
         * For debug purposes
         */
        //std::string toString() const;

    protected:
        IpcHeader m_header;
        uint8_t m_payload[MSG_MAX_SIZE];
        MsgSize m_currentPayloadSize;
        uint8_t* m_currentPayloadPosition;
};

} /* namespace arp_stm32 */
#endif /* DATAGRAM_HPP_ */
