/*
 * IpcTypes.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: ard
 */

#ifndef IPCTYPES_HPP_
#define IPCTYPES_HPP_

#include <stdint.h>
#include <utility>

//TODO
typedef unsigned char uint8_t;
//TODO
typedef unsigned short uint16_t;

namespace arp_stm32
{
    /**
     * This is a common pattern used for ram check.
     * In decimal it's 23130 (FYI, 0x5A is 90)
     * In binary it's 0101 1010 0101 1010
     *
     * We suppose that i's a pity if a real message can fake the magic number.
     * The communication could work without it, it's just a safety measure.
     * Of course a CRC would be better, but it increses complexity.
     */
    static const uint16_t MAGIC_NUMBER = 0x5A5A;

    typedef uint16_t MsgSize;
    typedef uint8_t MsgType;

    struct IpcHeaderPod
    {
            uint16_t    magic;
            MsgType     type;
            MsgSize     size;
    } __attribute((packed)); //without packed, the struct is aligned on uint16_t.

    static const MsgSize HEADER_SIZE = sizeof(IpcHeaderPod);
    static const MsgSize MSG_MAX_SIZE = 1000; //Take care to accord with USB_RX_BUFER_SIZE in usb.c

    typedef std::pair<uint8_t *, MsgSize> Payload;
    typedef std::pair<uint8_t const * const, MsgSize const> PayloadConst;

    //This is specific to linux but we'll survive to this shortcut ;p
    typedef int FileDesciptor;

}


#endif /* IPCTYPES_HPP_ */
