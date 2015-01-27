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

using namespace arp_stm32::ipc;

IpcMsg::IpcMsg(){}
IpcMsg::~IpcMsg(){}

bool IpcMsg::fillDatagram(Datagram& dtg) const
{
    Payload payload = dtg.getWritablePayload();
    bool res = serialize(payload);

    IpcHeader h;
    h.type = getType();
    h.magic = MAGIC_NUMBER;
    h.size = payload.second;

    dtg.setHeader(h);

    return res;
}

}
