/*
 * IpcMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "IpcMsg.hpp"
#include "Datagram.hpp"

namespace arp_stm32
{

IpcMsg::IpcMsg(){}
IpcMsg::~IpcMsg(){}

bool IpcMsg::fillDatagram(Datagram& dtg) const
{
    Payload payload = dtg.getWritablePayload();
    bool res = serialize(payload);
    dtg.extractPayload(getSize());

    IpcHeader h;
    h.type = getType();
    h.magic = MAGIC_NUMBER;
    h.size = getSize();

    dtg.setHeader(h);

    return res;
}

bool IpcMsg::fillDatagramAndHeader(Datagram& dtg, uint8_t * const headerBuffer) const
{
    Payload payload = dtg.getWritablePayload();
    bool res = serialize(payload);
    dtg.extractPayload(getSize());

    IpcHeader h;
    h.type = getType();
    h.magic = MAGIC_NUMBER;
    h.size = getSize();

    dtg.setHeader(h);

    res &= dtg.serializeHeader(headerBuffer);

    return res;
}

}
