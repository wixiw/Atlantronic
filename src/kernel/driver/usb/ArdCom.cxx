/*
 * ArdCom.cxx
 *
 *  Created on: Feb 1, 2015
 *      Author: robot
 */

#include "ArdCom.h"
#include "kernel/log.h"
#include <cstring>

using namespace std;
using namespace arp_stm32;


ArdCom* ArdCom::m_instance=0;

ArdCom::ArdCom()
	: m_dtg()
	, m_state(STATE_UNINITIALIZED)
{
}

void ArdCom::init()
{
	m_state = STATE_WAITING_HEADER;
}

ArdCom::~ArdCom() {
}


void ArdCom::registerMsgCallback(DiscoveryMsgType id, MsgCallback fct)
{
	m_msgCallbacks[id] = fct;
}

//copy a circular buffer into a linear buffer
void ArdCom::memcpy_circularToLinear(uint8_t* const linearBuffer, CircularBuffer const * const circularBuffer, MsgSize const sizeToCopy)
{
	int nMax = circularBuffer->size - circularBuffer->start;
	// mise "a plat" dans un seul buffer pour le traitement si necessaire
	if( sizeToCopy <= nMax )
	{
		// message deja contigu en memoire
		memcpy(linearBuffer, circularBuffer->data + circularBuffer->start, sizeToCopy);
	}
	else
	{
		memcpy(linearBuffer, 			circularBuffer->data + circularBuffer->start, 	nMax);
		memcpy(linearBuffer + nMax, 	circularBuffer->data, 							sizeToCopy - nMax);
	}
}

int ArdCom::waitingHeaderHook(CircularBuffer const * const buffer)
{
	//Si on a recu un message trop petit on attend la suite
	if( buffer->count < HEADER_SIZE )
	{
		log_format(LOG_ERROR, "protocol error, header size too short.");
		m_state = STATE_ERROR;
		return -1;
	}

	//decircularisation du buffer de reception
	memset(m_headerBuffer, 0, sizeof(m_headerBuffer));
	memcpy_circularToLinear( m_headerBuffer, buffer, HEADER_SIZE);

	//On a recu un message qui fait au moins la taille d'un header
	//on le deserialize pour connaitre la payload
    if( !m_dtg.deserializeHeader(m_headerBuffer) )
    {
    	log_format(LOG_ERROR, "protocol error, header deserialization failed.");
    	m_state = STATE_ERROR;
    	return -1;
    }

    m_state = STATE_WAITING_PAYLOAD;
    return HEADER_SIZE;
}

int ArdCom::waitingPayloadHook(CircularBuffer const * const buffer)
{
	//Si on a recu un message trop petit on attend la suite
	if( buffer->count < m_dtg.getHeader().size )
	{
		log_format(LOG_ERROR, "protocol error, payload size too short.");
		m_state = STATE_ERROR;
		return -1;
	}

	//decircularisation du buffer de reception
	memcpy_circularToLinear( m_dtg.getWritablePayload().first, buffer, m_dtg.getHeader().size);
	m_dtg.appendPayload(m_dtg.getHeader().size);

	//Call message callback associated to received header type.
	map<DiscoveryMsgType, MsgCallback>::iterator element = m_msgCallbacks.find(static_cast<DiscoveryMsgType>(m_dtg.getHeader().type));
	if( element == m_msgCallbacks.end() )
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", m_dtg.getHeader().type);
	else
		element->second(m_dtg);

    m_state = STATE_WAITING_HEADER;
	return m_dtg.getHeader().size;
}

int ArdCom::deserialize(CircularBuffer const * const buffer)
{
	int res = 0;

	switch(m_state)
	{
		case STATE_WAITING_HEADER:
			res = waitingHeaderHook(buffer);
			break;

		case STATE_WAITING_PAYLOAD:
			res = waitingPayloadHook(buffer);
			break;

		case STATE_UNINITIALIZED:
		case STATE_ERROR:
		default:
			res = -1;
			break;
	}

	return res;
}

