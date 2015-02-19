/*
 * HokuyoMessage.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "HokuyoMessage.hpp"
#include "DiscoveryIpcTypes.h"

namespace arp_stm32
{



HokuyoMessage::HokuyoMessage()
    : IpcMsg()
    , m_scan()
{
}

HokuyoMessage::HokuyoMessage(struct hokuyo_scan const& scan)
: IpcMsg()
, m_scan(scan)
{

}

HokuyoMessage::~HokuyoMessage(){}

bool HokuyoMessage::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, &m_scan, SIZE);
    payload.second = SIZE;
    return true;
}

bool HokuyoMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(&m_scan, payload.first, SIZE);
    return true;
}

MsgType HokuyoMessage::getType() const
{
    return MSG_HOKUYO_SCAN;
}

MsgSize HokuyoMessage::getSize() const
{
	return SIZE;
}

} /* namespace arp_stm32 */
