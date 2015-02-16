/*
 * ArdCom.cxx
 *
 *  Created on: Feb 1, 2015
 *      Author: robot
 */

#include "ArdCom.h"
#include "kernel/log.h"
#include <cstring>
#include "kernel/driver/usb.h"

using namespace std;
using namespace arp_stm32;

static bool firstTimeInError = false;

ArdCom* ArdCom::m_instance=0;

ArdCom::ArdCom()
	: m_recvDtg()
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
    if( !m_recvDtg.deserializeHeader(m_headerBuffer) )
    {
    	log_format(LOG_ERROR, "protocol error, header deserialization failed. magic=%x%x type=%d size=%d",
    	        m_headerBuffer[0],
    	        m_headerBuffer[1],
    	        m_headerBuffer[2],
    	        m_headerBuffer[3]+ ((m_headerBuffer[4])<<8));
    	m_state = STATE_ERROR;
    	return -1;
    }

    m_state = STATE_WAITING_PAYLOAD;
    return HEADER_SIZE;
}

int ArdCom::waitingPayloadHook(CircularBuffer const * const buffer)
{
	//Si on a recu un message trop petit on attend la suite
	if( buffer->count < m_recvDtg.getHeader().size )
	{
		log_format(LOG_ERROR, "protocol error, payload size too short.");
		m_state = STATE_ERROR;
		return -1;
	}

	//decircularisation du buffer de reception
	memcpy_circularToLinear( m_recvDtg.getWritablePayload().first, buffer, m_recvDtg.getHeader().size);
	m_recvDtg.appendPayload(m_recvDtg.getHeader().size);

	//Call message callback associated to received header type.
	map<DiscoveryMsgType, MsgCallback>::iterator element = m_msgCallbacks.find(static_cast<DiscoveryMsgType>(m_recvDtg.getHeader().type));
	if( element == m_msgCallbacks.end() )
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", m_recvDtg.getHeader().type);
	else
		element->second(m_recvDtg);

    m_state = STATE_WAITING_HEADER;
	return m_recvDtg.getHeader().size;
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

			if( ! firstTimeInError )
			{
				firstTimeInError = true;
				log_format(LOG_ERROR, "Unknown State.");
				res = -1;
			}
			break;
	}

	return res;
}

bool ArdCom::send(arp_stm32::IpcMsg& msg) const
{
	Datagram dtg;
	uint8_t headerBuffer[HEADER_SIZE];

	if(!msg.fillDatagramAndHeader(dtg,headerBuffer))
	{
		return false;
	}

	takeUsbMutex();

	usb_write(&headerBuffer, HEADER_SIZE);
	usb_write(dtg.getPayload().first, dtg.getPayload().second);

	signalUsbMsg();
	releaseUsbMutex();

	return true;
}

