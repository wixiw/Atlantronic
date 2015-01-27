/*
 * Datagram.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: willy
 */

#include "Datagram.hpp"
#include <cstring>

using namespace std;
using namespace arp_stm32;
using namespace arp_stm32::ipc;

Datagram::Datagram():
        m_header()
{
    m_currentPayloadSize = 0;
    m_currentPayloadPosition = m_payload;
    memset(m_payload, 0, MSG_MAX_SIZE);
}

Datagram::~Datagram()
{
}

bool Datagram::isPayloadFullyReceived() const
{
    return m_header.size == m_currentPayloadSize;
}

Payload Datagram::appendPayload(MsgSize size)
{
    if( m_header.size <= m_currentPayloadSize + size)
    {
        m_currentPayloadPosition = NULL;
        m_currentPayloadSize = m_header.size;
    }
    else
    {
        m_currentPayloadPosition += size;
        m_currentPayloadSize += size;
    }

    return std::make_pair(m_currentPayloadPosition, m_header.size - m_currentPayloadSize);
}

Payload Datagram::extractPayload(MsgSize size)
{
    if( m_currentPayloadSize - size <= 0)
    {
        m_currentPayloadPosition = NULL;
        m_currentPayloadSize = m_header.size;
    }
    else
    {
        m_currentPayloadPosition += size;
        m_currentPayloadSize -= size;
    }

    return std::make_pair(m_currentPayloadPosition, m_currentPayloadSize);
}

bool Datagram::deserializeHeader(uint8_t const * const buffer)
{
    if( m_header.deserialize(buffer) )
    {
        //reset the payload so that its ready to be received
        m_currentPayloadSize = 0;
        m_currentPayloadPosition = m_payload;
        memset(m_payload, 0, MSG_MAX_SIZE);
        return true;
    }
    else
    {
        return false;
    }
}

bool Datagram::serializeHeader(uint8_t * const buffer)
{
    if( m_header.serialize(buffer) )
    {
        //reset the payload so that its ready to be received
        m_currentPayloadSize = m_header.size;
        m_currentPayloadPosition = m_payload;
        memset(m_payload, 0, MSG_MAX_SIZE);
        return true;
    }
    else
    {
        return false;
    }
}

IpcHeader const& Datagram::getHeader() const
{
    return m_header;
}

void Datagram::setHeader(IpcHeader& h)
{
    m_header = h;
}

PayloadConst Datagram::getPayload() const
{
    if( isPayloadFullyReceived() )
    {
        return std::make_pair(m_payload, m_header.size);
    }
    else
    {
        return std::make_pair(m_payload, 0);
    }
}

Payload Datagram::getWritablePayload()
{
    return std::make_pair(m_payload, ipc::MSG_MAX_SIZE);
}

//std::string Datagram::toString() const
//{
//    return m_header.toString() + " " + uint8ToHexArray(m_payload,MSG_MAX_SIZE);
//}
