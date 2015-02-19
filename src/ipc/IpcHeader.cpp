/*
 * IpcHeader.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#include "IpcHeader.hpp"
#include <string>

using namespace arp_stm32;
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
            || 0 == type
            || MSG_MAX_SIZE < size)
    {
        return false;
    }

    //Update the data buffer <id,length,payload>
    IpcHeaderPod header;
    header.magic = magic;
    header.type = type;
    header.size = size;
    memcpy(buffer, &header, HEADER_SIZE);

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

