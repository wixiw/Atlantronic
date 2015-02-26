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
#include <stdlib.h>
#include "com/stack_com/IpcHeader.hpp"
#include "com/stack_com/Datagram.hpp"
#include "com/msgs/DiscoveryIpcMessage.hpp"
#include "linux_rw.hpp"

using namespace std;
using namespace arp_stm32;

#ifndef LINUX
#define LINUX
#endif



int main( int argc, char *argv[] )
{
	Datagram dtg;
	int readFd;

begin:
	readFd = openFd(argc, argv, O_RDONLY);
    if ( !readFd )
    {
    	cout << "Failed to open file => quit :(." << endl;
    	return -1;
    }

    while(1)
    {
    	if( !receiveMessage(dtg,readFd) )
    	{
    		goto begin;
    	}

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
        		//cout << "Status message received." << endl;
        		break;

        	case MSG_X86_CMD:
        	{
        		X86CmdMsg msg;
        		if( msg.deserialize(dtg.getPayload()) )
        		{
        			cout << "X86 cmd received." << endl;
        		}
        		else
        		{
        			cout << "Failed to deserialize x86cmd message." << endl;
        		}
        		break;
        	}

        	default:
        		cout << "Unknown message type (" << static_cast<unsigned int>(dtg.getHeader().type) << ")" << endl;
        		break;
        }

    }
}
