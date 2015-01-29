/*
 * OpponentListMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "OpponentListMsg.hpp"
#include "DiscoveryIpcTypes.hpp"
#include <sstream>
#include <iostream>

namespace arp_stm32
{

using namespace arp_stm32::ipc;
using namespace std;

OpponentListMsg::OpponentListMsg()
    : IpcMsg()
    , m_oppList()
{}

OpponentListMsg::~OpponentListMsg(){}

bool OpponentListMsg::serialize(Payload& payload) const
{
    MsgSize expectedSize = m_oppList.size() * sizeof(detection_object);
    if( MSG_MAX_SIZE < expectedSize)
    {
        payload.second = 0;
        return false;
    }

    uint8_t* currentPos = payload.first;
    list<detection_object>::const_iterator i;
    detection_object obj;
    for( i = m_oppList.begin() ; i != m_oppList.end() ; i++)
    {
        obj = *i;
        memcpy(currentPos, &obj, sizeof(detection_object));
        currentPos += sizeof(detection_object);
    }

    if(currentPos - payload.first != expectedSize)
    {
        payload.second = 0;
        return false;
    }

    payload.second = expectedSize;
    return true;
}

bool OpponentListMsg::deserialize(PayloadConst payload)
{
    if( payload.second % sizeof(detection_object) != 0
            || payload.first == NULL)
    {
        cout << "Size = " << payload.second << " modulo=" << sizeof(detection_object) << " res=" << payload.second % sizeof(detection_object);
        return false;
    }
    m_oppList.clear();

    int16_t nbOpp = payload.second / sizeof(detection_object);
    detection_object obj;
    uint8_t const* currentPos = payload.first;
    for( int i = 0 ; i < nbOpp ; i++)
    {
        memcpy(&obj, currentPos, sizeof(detection_object));
        m_oppList.push_back(obj);
        currentPos += sizeof(detection_object);
    }

    return true;
}

string OpponentListMsg::toString() const
{
    ostringstream os;
    os << "OppList(" << m_oppList.size() << ")[x,y,size] = [";
    std::list<detection_object>::const_iterator i;
    detection_object obj;
    for( i = m_oppList.begin() ; i != m_oppList.end() ; i++)
    {
        obj = *i;
        os << obj.x << "," << obj.y << ", " << obj.size << "|";
    }
    os << "]";
    return os.str();
}

std::list<detection_object> const & OpponentListMsg::getOppList() const
{
    return m_oppList;
}

MsgType OpponentListMsg::getType() const
{
    return MSG_OPP_LIST;
}

void OpponentListMsg::addOpponent(detection_object& opp)
{
    m_oppList.push_back(opp);
}

} /* namespace arp_stm32 */
