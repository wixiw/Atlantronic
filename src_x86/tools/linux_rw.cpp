/*
 * linux_rw.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#include "linux_rw.hpp"
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include "com/msgs/DiscoveryIpcMessage.hpp"

using namespace std;
using namespace arp_stm32;

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
	cout << "OpponentListMsg : \tchanging" << endl;
	StatusMessage msgStatus;
	cout << "StatusMessage : \t" << msgStatus.getSize() << endl;
	VersionMessage msgVersion;
	cout << "VersionMessage : \t" << msgVersion.getSize() << endl;
	X86CmdMsg msgCmd;
	cout << "X86CmdMsg : \t\t" << msgCmd.getSize() << endl;
	cout << "----------------------------------------------------" << endl;
}

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

bool receiveMessage(Datagram& dtg, int readFd)
{
    uint8_t buffer[MSG_MAX_SIZE];
    memset(buffer, 0, MSG_MAX_SIZE);
    int readBytes = 0;
    while( readBytes < HEADER_SIZE)
    {
    	int readSize = 0;
    	readSize = ::read(readFd, buffer+readBytes, HEADER_SIZE-readBytes);
    	if( readSize <= 0 )
    	{
    		cout << "File closed" << endl;
    		return false;
    	}
   		readBytes += readSize;
   		//cout << "Read H : " << readBytes << "/" << HEADER_SIZE << endl;
    }

    if( readBytes == HEADER_SIZE && dtg.deserializeHeader(buffer) )
    {
        //cout << "Deserialized header successfully, type=" << MessagePrinter::toString(dtg.getHeader()) << endl;
    }
    else
    {
        cout << "Failed to deserialize header nb bytes=" << readBytes << " buf=" << MessagePrinter::uint8ToHexArray(buffer, 5) << endl;
        return false;
    }

    memset(buffer, 0, MSG_MAX_SIZE);

    Payload p = dtg.appendPayload(0);
    readBytes = 0;
    while( NULL != p.first)
    {
    	//cout << "Read P : " << p.second << "/" << dtg.getHeader().size << endl;
    	readBytes = ::read(readFd, p.first, p.second);
    	if( readBytes <= 0 )
    	{
    		cout << "File closed" << endl;
    		return false;
    	}
    	p = dtg.appendPayload(readBytes);
    }

    //cout << "Datagram received : " << MessagePrinter::uint8ToHexArray(dtg.getPayload().first, dtg.getPayload().second )<< endl;
    return true;
}

int openFd( int argc, char *argv[], mode_t mode)
{
    if ( argc > 2 ) /* argc should be 2 for correct execution */
    {
        /* We print argv[0] assuming it is the program name */
       cout << "Wrong parameter : usage: " << argv[0] << " filename" << endl;
       exit(-1);
    }
    if( argc == 1 )
    {
		cout << "Opening : /dev/stm32_ard" << endl;

		int writeFd = 0;
		while( writeFd <= 0 )
		{
			writeFd = ::open("/dev/stm32_ard", mode);
			if (writeFd < 0)
			{
				cout << "Open returned an error" << strerror(errno) << endl;
				cout << "Retry in 1s ..." << endl;
				sleep(1);
				return -1;
			}
		}

		return writeFd;
    }
    else
    {
		cout << "Opening : " << argv[1] << endl;

		int writeFd = 0;
		while( writeFd <= 0 )
		{
			writeFd = ::open(argv[1], mode);
			if (writeFd < 0)
			{
				cout << "Open returned an error" << strerror(errno) << endl;
				cout << "Retry in 1s ..." << endl;
				sleep(1);
				return -1;
			}
		}

		return writeFd;
    }
}

