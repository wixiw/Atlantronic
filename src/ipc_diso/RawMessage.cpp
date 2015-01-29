/*
 * RawMessage.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#include "RawMessage.hpp"
#include <string>
#include <math/math.hpp>

using namespace arp_stm32;
using namespace arp_stm32::ipc;
using namespace std;

RawMessage::RawMessage()
    : IpcMsg()
    , m_payloadSize(0)
{
    memset(m_data, 0, sizeof(m_data));
}

bool RawMessage::serialize(Payload& payload) const
{
    if( payload.first == NULL)
    {
        payload.second = 0;
        return false;
    }

    memcpy(payload.first, m_data, m_payloadSize);
    payload.second = m_payloadSize;
    return true;
}

bool RawMessage::deserialize(PayloadConst payload)
{
    if( NULL == payload.first )
    {
        return false;
    }
    setPayload(payload.first, payload.second);
    return true;
}

uint8_t const* RawMessage::getPayload() const
{
    return m_data;
}

void RawMessage::setPayload(uint8_t const * const data, MsgSize size)
{
    if( NULL == data )
    {
        memset(m_data, 0, MSG_MAX_SIZE);
    }

    memcpy(m_data, data, size);
    m_payloadSize = size;
}

MsgType RawMessage::getType() const
{
    return 666U;
}

string RawMessage::toString() const
{
    using namespace arp_math;
    ostringstream s;
     s << "RawMsg = " << uint8ToHexArray(m_data, ipc::MSG_MAX_SIZE) << ".";
    return s.str();
}
