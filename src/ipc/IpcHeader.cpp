/*
 * IpcHeader.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#include "IpcHeader.hpp"
#include <cstring>

using namespace arp_stm32;
using namespace arp_stm32::ipc;
using namespace std;

IpcHeader::IpcHeader():
        magic(MAGIC_NUMBER),
        type(0),
        size(0)
{
}

bool IpcHeader::serialize(uint8_t* const buffer) const
{
    if( NULL == buffer
            || 0 == type )
    {
        return false;
    }

    //Update the data buffer <id,length,payload>
    IpcHeaderPod header;
    header.magic = magic;
    header.type = type;
    header.size = size;
    memcpy(buffer, &header, HEADER_SIZE);
    MsgSize size=0;
    if( MSG_MAX_SIZE < size
            || size != size)
    {
        return false;
    }

    return true;
}

bool IpcHeader::deserialize(uint8_t const * const buffer)
{
    if( NULL == buffer )
    {
        return false;
    }

    IpcHeaderPod const * const header = reinterpret_cast<IpcHeaderPod const * const>(buffer);

    if( NULL == header
            || header->magic != MAGIC_NUMBER
            || MSG_MAX_SIZE  < header->size
            || 0 == header->type)
    {
        return false;
    }

    magic = header->magic;
    type = header->type;
    size = header->size;
    return true;
}

//string IpcHeader::toString() const
//{
//    ostringstream s;
//    s << "magic=" << hex << magic << " id=0x" << dec <<  static_cast<unsigned int>(type) << " l=" << static_cast<unsigned int>(size);
//    return s.str();
//}
