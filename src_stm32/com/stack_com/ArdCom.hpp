/*
 * ArdCom.h
 *
 *  Created on: Feb 1, 2015
 *      Author: willy
 */

#ifndef ARDCOM_H_
#define ARDCOM_H_

#include "com/stack_com/IpcMsg.hpp"
#include "com/stack_com/Datagram.hpp"
#include "com/usb/circular_buffer.h"
#include "com/msgs/DiscoveryIpcTypes.h"

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
		STATE_RESYNC,
		STATE_ERROR
	} comState;

	static ArdCom& getInstance();

	bool send(arp_stm32::IpcMsg& msg) const;

private:
	ArdCom();

	//try to deserialize the header which is a fixed size message
	//@return nb bytes read on success, negative number on error.
	int waitingHeaderHook(CircularBuffer * const buffer);

	int waitingPayloadHook(CircularBuffer * const buffer);

	//In case something bad happened in communication, this state will
	//unqueue any byte that is not part of the magic number.
	//When the magic number is received, then we will return to a header waiting state
	int resyncHook(CircularBuffer * const buffer);

	void msgCb_event(arp_stm32::Datagram& dtg);


	arp_stm32::Datagram m_recvDtg;
	MsgCallback m_msgCallbacks[MSG_NB];
	comState m_state;
	uint8_t m_headerBuffer[arp_stm32::HEADER_SIZE];
	static ArdCom* m_instance;
	size_t m_resyncMagicByteNb;
};

#endif /* ARDCOM_H_ */
