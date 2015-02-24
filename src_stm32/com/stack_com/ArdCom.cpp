/*
 * ArdCom.cxx
 *
 *  Created on: Feb 1, 2015
 *      Author: robot
 */

#include "ArdCom.hpp"
#include "components/log/log.h"
#include "com/usb/usb.h"

using namespace arp_stm32;

ArdCom* ArdCom::m_instance=0;

ArdCom::ArdCom()
	: m_recvDtg()
	, m_state(STATE_UNINITIALIZED)
	, m_resyncMagicByteNb(0)
{
	for( int i = 0 ; i < MSG_NB ; i++ )
	{
		m_msgCallbacks[i] = NULL;
	}
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

int ArdCom::waitingHeaderHook(CircularBuffer * const buffer)
{
	//Si on a recu un message trop petit on attend la suite
	if( circular_getOccupiedRoom(buffer) < HEADER_SIZE - m_resyncMagicByteNb )
	{
		return 0;
	}

	if( m_resyncMagicByteNb == 0 )
	{
		memset(m_headerBuffer, 0, sizeof(m_headerBuffer));
	}

	//decircularisation du buffer de reception
	circular_pop( m_headerBuffer + m_resyncMagicByteNb, buffer, HEADER_SIZE - m_resyncMagicByteNb);

	//On a recu un message qui fait au moins la taille d'un header
	//on le deserialize pour connaitre la payload
    if( !m_recvDtg.deserializeHeader(m_headerBuffer) )
    {
    	log_format(LOG_ERROR, "protocol error, header deserialization failed : magic=%02X%02X type=%d size=%d. Going to resync...",
    	        m_headerBuffer[0],
    	        m_headerBuffer[1],
    	        m_headerBuffer[2],
    	        m_headerBuffer[3]+ ((m_headerBuffer[4])<<8));
    	m_state = STATE_RESYNC;
    	return -1;
    }

    //If we start the deserialization with a pre-fetched magic number, we consider the communication re-established
    if( m_resyncMagicByteNb != 0 )
    {
    	m_resyncMagicByteNb = 0;
    }

    m_state = STATE_WAITING_PAYLOAD;
    return HEADER_SIZE;
}

int ArdCom::waitingPayloadHook(CircularBuffer * const buffer)
{
	//Si on a recu un message trop petit on attend la suite
	if( circular_getOccupiedRoom(buffer) < m_recvDtg.getHeader().size )
	{
		return 0;
	}

	//decircularisation du buffer de reception
	circular_pop( m_recvDtg.getWritablePayload().first, buffer, m_recvDtg.getHeader().size);
	m_recvDtg.appendPayload(m_recvDtg.getHeader().size);

	//Call message callback associated to received header type.
	if( m_recvDtg.getHeader().type < MSG_NB && m_msgCallbacks[m_recvDtg.getHeader().type] != NULL )
	{
		m_msgCallbacks[m_recvDtg.getHeader().type](m_recvDtg);
	}
	else
	{
		log_format(LOG_ERROR, "protocol error, unknown message type=%d.", m_recvDtg.getHeader().type);
	}

    m_state = STATE_WAITING_HEADER;
	return m_recvDtg.getHeader().size;
}

int ArdCom::resyncHook(CircularBuffer * const buffer)
{
	size_t readByteNb = 0;

	//On ne reset le buffer que si on commence le resync, sinon on peut avoir fini avec un bout de magic dans le tour précédent
	if( m_resyncMagicByteNb == 0 )
	{
		memset(m_headerBuffer, 0, sizeof(m_headerBuffer));
	}

	//Tant qu'il y a des nouveaux octets et qu'on a pas trouvé les 2 octets magiques :
	while( 0 < circular_getOccupiedRoom(buffer) && m_resyncMagicByteNb < sizeof(IpcHeader::magic) )
	{
		//on lit le premier byte :
		circular_pop( m_headerBuffer + m_resyncMagicByteNb, buffer, 1);
		if( m_headerBuffer[m_resyncMagicByteNb] == 0x5A ) //TODO voir comment on peut lier à MAGIC_NUMBER
		{
			m_resyncMagicByteNb++;
		}

		readByteNb++;
	}


	//On a recu un message qui fait au moins la taille d'un header
	//on le deserialize pour connaitre la payload
    if( m_resyncMagicByteNb == sizeof(IpcHeader::magic))
    {
    	log_format(LOG_INFO, "Communication re-sync'ed");
    	m_state = STATE_WAITING_HEADER;
    }

    return readByteNb;
}

int ArdCom::deserialize(CircularBuffer * const buffer)
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

		case STATE_RESYNC:
			res = resyncHook(buffer);
			break;
		case STATE_UNINITIALIZED:
		case STATE_ERROR:
		default:
			log_format(LOG_ERROR, "Unknown State.");
			res = -1;
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

	take_txUsbMutex();

	usb_write(&headerBuffer, HEADER_SIZE);
	usb_write(dtg.getPayload().first, dtg.getPayload().second);

	release_txUsbMutex();

	signal_txUsbMsg();

	return true;
}

ArdCom& ArdCom::getInstance()
{
	take_txUsbMutex();
	if( 0 == m_instance )
	{
		m_instance = new ArdCom();
	}
	release_txUsbMutex();
	return *m_instance;
}

