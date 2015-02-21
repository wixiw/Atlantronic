/*
 * ArdCom.h
 *
 *  Created on: Feb 1, 2015
 *      Author: willy
 */

#ifndef ARDCOM_H_
#define ARDCOM_H_

#include "ipc/IpcMsg.hpp"
#include "ipc/Datagram.hpp"
#include "ipc_disco/DiscoveryIpcTypes.h"
#include "circular_buffer.h"

class ArdCom {
public:
	typedef void (*MsgCallback)(arp_stm32::Datagram& dtg);

	virtual ~ArdCom();

	int deserialize(CircularBuffer * const buffer);

	void registerMsgCallback(DiscoveryMsgType id, MsgCallback fct);

	void init();

	typedef enum{
		STATE_UNINITIALIZED,
		STATE_WAITING_HEADER,
		STATE_WAITING_PAYLOAD,
		STATE_ERROR
	} comState;

	static ArdCom& getInstance()
	{
		if( 0 == m_instance )
		{
			m_instance = new ArdCom();
		}
		return *m_instance;
	}

	bool send(arp_stm32::IpcMsg& msg) const;

private:
	ArdCom();

	//try to deserialize the header which is a fixed size message
	//@return nb bytes read on success, negative number on error.
	int waitingHeaderHook(CircularBuffer * const buffer);

	int waitingPayloadHook(CircularBuffer * const buffer);

	void msgCb_event(arp_stm32::Datagram& dtg);


	arp_stm32::Datagram m_recvDtg;
	MsgCallback m_msgCallbacks[MSG_NB];
	comState m_state;
	uint8_t m_headerBuffer[arp_stm32::HEADER_SIZE];
	static ArdCom* m_instance;
};

#endif /* ARDCOM_H_ */
