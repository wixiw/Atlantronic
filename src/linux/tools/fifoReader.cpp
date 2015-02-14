/*
 * fifoReader.cpp
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
#include "ipc/IpcHeader.hpp"
#include "ipc/Datagram.hpp"
#include "ipc_disco/MessagePrinter.hpp"
#include "ipc_disco/RawMessage.hpp"
#include "ipc_disco/DiscoveryIpcTypes.h"
#include "ipc_disco/VersionMessage.hpp"
#include "ipc_disco/StatusMessage.hpp"
#include "ipc_disco/LogMessage.hpp"
#include "ipc_disco/FaultMessage.hpp"
#include "ipc_disco/EventMessage.hpp"
#include "ipc_disco/OpponentListMsg.hpp"

using namespace std;
using namespace arp_stm32;

#ifndef LINUX
#define LINUX
#endif

int main()
{
    //Try to open the file, this call is blocking if no reader are present
    int readFd = ::open("/tmp/carte.out", O_RDONLY);
    if( readFd < 0 )
    {
        cout << "Open returned an error : " << strerror(errno) << endl;
        return -1;
    }

    cout << "Open success, Waiting data." << endl;

    while(1)
    {
        uint8_t buffer[MSG_MAX_SIZE];
        memset(buffer, 0, MSG_MAX_SIZE);
        int readBytes = 0;
        while( readBytes < HEADER_SIZE)
        {
        	int readSize = 0;
        	readSize = ::read(readFd, buffer+readBytes, HEADER_SIZE-readBytes);
       		readBytes += readSize;
       		//cout << "Read H : " << readBytes << "/" << HEADER_SIZE << endl;
        }

        Datagram dtg;

        if( readBytes == HEADER_SIZE && dtg.deserializeHeader(buffer) )
        {
            //cout << "Deserialized header successfully, type=" << MessagePrinter::toString(dtg.getHeader()) << endl;
        }
        else
        {
            cout << "Failed to deserialize header nb bytes=" << readBytes << " buf=" << MessagePrinter::uint8ToHexArray(buffer, 5) << endl;
            continue;
        }

        memset(buffer, 0, MSG_MAX_SIZE);

        Payload p = dtg.appendPayload(0);
        readBytes = 0;
        while( NULL != p.first)
        {
        	//cout << "Read P : " << p.second << "/" << dtg.getHeader().size << endl;
        	readBytes = ::read(readFd, p.first, p.second);
        	p = dtg.appendPayload(readBytes);
        }

        //cout << "Datagram received : " << MessagePrinter::uint8ToHexArray(dtg.getPayload().first, dtg.getPayload().second )<< endl;

        switch(dtg.getHeader().type)
        {
        	case MSG_VERSION:
        	{
        		VersionMessage msg;
        		if( msg.deserialize(dtg.getPayload()) )
        		{
        			cout << "Version message received : " << msg.getVersion() << endl;
        		}
        		else
        		{
        			cout << "Failed to deserialize Version message." << endl;
        		}
        		break;
        	}

        	case MSG_LOG:
        	{
        		LogMessage msg;
        		if( msg.deserialize(dtg.getPayload()) )
        		{
        			cout << "[LOG:" << msg.getLevelText() << "] " << msg.getLogText() << endl;
        		}
        		else
        		{
        			cout << "Failed to deserialize Log message." << endl;
        		}
        		break;
        	}

        	case MSG_EVENT:
        	{
        		EventMessage msg;
        		if( msg.deserialize(dtg.getPayload()) )
        		{
        			cout << "Event message received, eventId=" << msg.getEventId() << endl;
        		}
        		else
        		{
        			cout << "Failed to deserialize Event message." << endl;
        		}
        		break;
        	}

        	case MSG_STATUS:
        		//not displayed as oftently received
        		break;

        	default:
        		cout << "Unknown message type (" << static_cast<unsigned int>(dtg.getHeader().type) << ")" << endl;
        		break;
        }

    }
}
