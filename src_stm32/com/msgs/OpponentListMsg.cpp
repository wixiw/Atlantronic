/*
 * OpponentListMsg.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#include "OpponentListMsg.hpp"
#include "DiscoveryIpcTypes.h"

namespace arp_stm32
{

OpponentListMsg::OpponentListMsg(detection_object const * const objs, uint8_t const obj_size)
    : IpcMsg()
	, m_oppListSize(obj_size)
{
	memset(m_oppList, 0, DETECTION_NUM_OBJECT);
	if( obj_size < DETECTION_NUM_OBJECT)
	{
		memcpy(m_oppList, objs, obj_size*sizeof(detection_object));
	}
}

OpponentListMsg::OpponentListMsg()
	: IpcMsg()
	, m_oppListSize(0)
{
	memset(m_oppList, 0, DETECTION_NUM_OBJECT);
}

OpponentListMsg::~OpponentListMsg(){}

bool OpponentListMsg::serialize(Payload& payload) const
{
    if( MSG_MAX_SIZE < getSize())
    {
        payload.second = 0;
        return false;
    }

    uint8_t* currentPos = payload.first;

    memcpy(currentPos, &m_oppListSize, sizeof(m_oppListSize));
    currentPos += m_oppListSize;

	memcpy(currentPos, &m_oppList, m_oppListSize*sizeof(detection_object));
	currentPos += m_oppListSize*sizeof(detection_object);

    if(currentPos - payload.first != getSize())
    {
        payload.second = 0;
        return false;
    }

    payload.second = getSize();
    return true;
}

bool OpponentListMsg::deserialize(PayloadConst payload)
{
    if( payload.second % sizeof(detection_object) != 0
            || payload.first == NULL)
    {
        //cout << "Size = " << payload.second << " modulo=" << sizeof(detection_object) << " res=" << payload.second % sizeof(detection_object);
        return false;
    }

    uint8_t const * currentPos = payload.first;


    memcpy(&m_oppListSize, currentPos, sizeof(m_oppListSize));
    currentPos += m_oppListSize;


    memset( m_oppList, 0, DETECTION_NUM_OBJECT);

    memcpy(m_oppList, currentPos, m_oppListSize*sizeof(detection_object));
    currentPos += m_oppListSize*sizeof(detection_object);

    return true;
}

detection_object const * OpponentListMsg::getOppList() const
{
    return m_oppList;
}

MsgType OpponentListMsg::getType() const
{
    return MSG_OPP_LIST;
}

MsgSize OpponentListMsg::getSize() const
{
	return sizeof(m_oppListSize) + m_oppListSize * sizeof(detection_object);
}

bool OpponentListMsg::addOpponent(detection_object& opp)
{
	if( m_oppListSize < DETECTION_NUM_OBJECT )
	{
		memcpy(&(m_oppList[m_oppListSize]), &opp, sizeof(detection_object));
		m_oppListSize++;
		return true;
	}
	else
	{
		return false;
	}
}

} /* namespace arp_stm32 */
