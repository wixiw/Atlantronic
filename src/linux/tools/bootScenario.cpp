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
#include "discovery/boot_signals.h"
#include "ipc/IpcHeader.hpp"
#include "ipc_disco/DiscoveryIpcMessage.hpp"

using namespace std;
using namespace arp_stm32;

#ifndef LINUX
#define LINUX
#endif

void sendMsg(IpcMsg& msg, int writeFd)
{
    Datagram dtg;
    uint8_t headerBuffer[HEADER_SIZE];
    if( ! msg.fillDatagramAndHeader(dtg, headerBuffer) )
    {
    	cout << "failed to serialize msg" << endl;
    	return;
    }

   int writeSize = 0;
    while( writeSize != HEADER_SIZE)
    {
    	writeSize += ::write(writeFd, headerBuffer+writeSize , HEADER_SIZE-writeSize);
    	cout << " written : " << writeSize << endl;
    	usleep(50);
    }



    writeSize = 0;
    Payload p = dtg.extractPayload(0);
     while( p.first != NULL)
     {
    	 writeSize += ::write(writeFd, dtg.getPayload().first, dtg.getPayload().second);
    	 cout << " written : " << writeSize << endl;
    	 p = dtg.extractPayload(writeSize);
    	 usleep(50);
     }
}

int openFd()
{
    //Try to open the file, this call is blocking if no reader are present
    int writeFd = ::open("/tmp/carte.in", O_WRONLY);
    if( writeFd < 0 )
    {
        cout << "Open returned an error" << strerror(errno) << endl;
        return -1;
    }
    return writeFd;
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

int main()
{
	displayMsgSizes();

	int writeFd = openFd();

    ConfigurationMsg msgConfig;
    msgConfig.setMatchDuration(5.0);
	msgConfig.setModuleStartConfig( BOOT_ID_CONTROL, true);
	msgConfig.setModuleStartConfig( BOOT_ID_DETECTION, true);
	msgConfig.setModuleStartConfig( BOOT_ID_FAULT, false);//TODO bugged
	msgConfig.setModuleStartConfig( BOOT_ID_DYNAMIXEL, true);
	msgConfig.setModuleStartConfig( BOOT_ID_HOKUYO, true);
	msgConfig.setControlPeriod(200);
    sendMsg(msgConfig, writeFd);


            char line[100];
            cout << "hit any key to send cmd..." << endl;
            std::cin.getline(line, 100);

    int i = 0;
    while(1)
    {
//        char line[100];
//        cout << "hit any key to send cmd..." << endl;
//        std::cin.getline(line, 100);


    	X86CmdMsg msgCmd;
    	sendMsg(msgCmd, writeFd);

    	if( i%10 == 0 )
    	{
    		EventMessage msgEvt(EVT_LIST_TASKS);
        	sendMsg(msgEvt, writeFd);
    	}
    	i++;
    	cout << "i=" << i << endl;
    	usleep(1500*1E3);
    }
}


