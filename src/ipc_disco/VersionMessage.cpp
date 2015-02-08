/*
 * VersionMessage.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#include "VersionMessage.hpp"
#include "DiscoveryIpcTypes.h"
#include <cstring>

using namespace arp_stm32;
using namespace std;

const char VersionMessage::EXPECTED_VERSION[VersionMessage::SIZE] = VERSION;

VersionMessage::VersionMessage():
        IpcMsg()
{
    memset(m_version, '?', SIZE);
}

bool VersionMessage::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < SIZE)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, EXPECTED_VERSION, SIZE);
    payload.second = SIZE;
    return true;
}

bool VersionMessage::deserialize(PayloadConst payload)
{
    if( payload.second != SIZE
            || MSG_MAX_SIZE < SIZE
            || payload.first == NULL)
    {
        return false;
    }

    memcpy(m_version, payload.first, SIZE);
    return true;
}

char const* VersionMessage::getVersion() const
{
    return m_version;
}

void VersionMessage::setVersion(char const * const version)
{
    memcpy(m_version, version, SIZE);
}

MsgType VersionMessage::getType() const
{
    return MSG_VERSION;
}

MsgSize VersionMessage::getSize() const
{
	return SIZE;
}
