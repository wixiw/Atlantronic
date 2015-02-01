/*
 * IpcHeader.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef IPCHEADER_HPP_
#define IPCHEADER_HPP_

#include "IpcTypes.hpp"

namespace arp_stm32
{

/*
 * The header is composed of :
 * _ a magic number, used for safety purposes
 * _ a message type to identify it
 * _ a message length to size the following payload
 */
class IpcHeader
{
    //everything is public as it's a POD class.
    public:
        uint16_t        magic;
        MsgType    type;
        MsgSize    size;


        IpcHeader();
        ~IpcHeader(){};

        /**
         * Update data buffer from class members. It will call serializePayload() on child.
         * @param buffer : memory into which the serialization is done.
         *                 The size of the buffer must be exactly HEADER_SIZE.
         * @return : false if : the provided pointer is NULL
         *                      or the type of the msg is 0, meaning it has not been initialized,
         *           rue in any other case, meaning the serialization is done properly.
         */
        bool serialize(uint8_t* const buffer) const;

        /**
         * Update Msg class member from header buffer
         * (as UsbMessages may have different length, the reception is done in 2 part : the header first, then the data)
         * @param buffer : the received buffer containing the header to be interpreted.
         *                 The size of the buffer must be exactly HEADER_SIZE.
         * @return : false if either : the headerSize is not HEADER_SIZE,
         *                             or header decoded length is greather than MSG_MAX_SIZE,
         *                             or the magic number is not correct
         *                             or the provided pointer is null
         *           true in any other case, meaning the deserialization is done properly.
         *
         */
        bool deserialize(uint8_t const * const buffer);

        /**
         * For debug purposes
         */
//        std::string toString() const;
};

} /* namespace arp_stm32 */
#endif /* IPCHEADER_HPP_ */
