/*
 * fifoWriter.cpp
 *
 *  Created on: Dec 28, 2014
 *      Author: willy
 */

#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include "discovery/boot_signals.h"
#include "com/stack_com/IpcHeader.hpp"
#include "com/msgs/DiscoveryIpcMessage.hpp"

using namespace std;
using namespace arp_stm32;

#ifndef LINUX
#define LINUX
#endif

void sendMsg(IpcMsg& msg, int writeFd)
{
	Datagram dtg;
	uint8_t headerBuffer[HEADER_SIZE];
	if (!msg.fillDatagramAndHeader(dtg, headerBuffer))
	{
		cout << "failed to serialize msg" << endl;
		return;
	}

	int writeSize = 0;
	while (writeSize != HEADER_SIZE)
	{
		int bytes = ::write(writeFd, headerBuffer + writeSize, HEADER_SIZE - writeSize);
		if (bytes < 0)
		{
			cout << "ERROR : negative write" << endl;
			while (1);
		}

		writeSize += bytes;
		cout << " written : " << writeSize << endl;
		usleep(50);
	}

	writeSize = 0;
	Payload p = dtg.extractPayload(0);
	while (p.first != NULL)
	{
		int bytes = ::write(writeFd, dtg.getPayload().first, dtg.getPayload().second);
		if (bytes < 0)
		{
			cout << "ERROR : negative write" << endl;
			while (1);
		}
		writeSize += bytes;
		cout << " written : " << writeSize << endl;
		p = dtg.extractPayload(writeSize);
		usleep(50);
	}
}

void sendMsgWithPause(IpcMsg& msg, int writeFd)
{
	sendMsg(msg, writeFd);

	char line[100];
	cout << "hit enter to send cmd..." << endl;
	std::cin.getline(line, 100);
}

int openFd( int argc, char *argv[] )
{
    if ( argc != 2 ) /* argc should be 2 for correct execution */
    {
        /* We print argv[0] assuming it is the program name */
       cout << "Wrong parameter : usage: " << argv[0] << " filename" << endl;
       exit(-1);
    }
    else
    {
		cout << "Opening : " << argv[1] << endl;

		int writeFd = ::open(argv[1], O_WRONLY);
		if (writeFd < 0)
		{
			cout << "Open returned an error" << strerror(errno) << endl;
			return -1;
		}
		return writeFd;
    }
}

void displayMsgSizes()
{
	cout << "----------------------------------------------------" << endl;
	ConfigurationMsg msgConfig;
	cout << "ConfigurationMsg : \t" << msgConfig.getSize() << endl;
	EventMessage msgEvent;
	cout << "EventMsg : \t\t" << msgEvent.getSize() << endl;
	FaultMessage msgFault;
	cout << "FaultMessage : \t\t" << msgFault.getSize() << endl;
	GyroMsg msgGyro;
	cout << "GyroMsg : \t\t" << msgGyro.getSize() << endl;
	HokuyoMessage msgHky;
	cout << "HokuyoMessage : \t" << msgHky.getSize() << endl;
	OpponentListMsg msgOpp;
	cout << "OpponentListMsg : \t" << msgOpp.getSize() << endl;
	StatusMessage msgStatus;
	cout << "StatusMessage : \t" << msgStatus.getSize() << endl;
	VersionMessage msgVersion;
	cout << "VersionMessage : \t" << msgVersion.getSize() << endl;
	X86CmdMsg msgCmd;
	cout << "X86CmdMsg : \t\t" << msgCmd.getSize() << endl;
	cout << "----------------------------------------------------" << endl;
}

int main ( int argc, char *argv[] )
{
	int writeFd = openFd(argc, argv);

	displayMsgSizes();

	char line[100];
	cout << "hit enter to start..." << endl;
	std::cin.getline(line, 100);

	EventMessage msgReboot(EVT_REBOOT);
	cout << "Reboot request sended." << endl;
	sendMsgWithPause(msgReboot, writeFd);

	ConfigurationMsg msgConfig;
	msgConfig.setMatchDuration(5.0);
	msgConfig.setModuleStartConfig(BOOT_ID_CONTROL, true);
	msgConfig.setModuleStartConfig(BOOT_ID_DETECTION, true);
	msgConfig.setModuleStartConfig(BOOT_ID_FAULT, false);    //TODO bugged
	msgConfig.setModuleStartConfig(BOOT_ID_DYNAMIXEL, true);
	msgConfig.setModuleStartConfig(BOOT_ID_HOKUYO, true);
	msgConfig.setControlPeriod(200);
	cout << "Config request sended." << endl;
	sendMsgWithPause(msgConfig, writeFd);


	EventMessage msgx86initOk(EVT_X86_INIT_DONE);
	cout << "X86 booted event sended." << endl;
	sendMsgWithPause(msgx86initOk, writeFd);

	EventMessage msgx86readyForMatch(EVT_X86_READY_FOR_MATCH);
	cout << "Ready for match sended." << endl;
	sendMsgWithPause(msgx86readyForMatch, writeFd);


	int i = 0;
	while (1) {
		X86CmdMsg msgCmd;
		sendMsg(msgCmd, writeFd);

		if (i % 10 == 0) {
			EventMessage msgEvt(EVT_LIST_TASKS);
			sendMsg(msgEvt, writeFd);
		}
		i++;
		cout << "i=" << i << endl;
		//usleep(1 * 1E3);
	}
}

