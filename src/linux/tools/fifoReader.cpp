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
        int readBytes = ::read(readFd, buffer, HEADER_SIZE);
        Datagram dtg;

        if( readBytes == HEADER_SIZE && dtg.deserializeHeader(buffer) )
        {
            cout << "Deserialized buffer successfully" << endl;
        }
        else
        {
            cout << "Failed to deserialize header nb bytes=" << readBytes << endl;
            continue;
        }

        memset(buffer, 0, MSG_MAX_SIZE);
        readBytes = ::read(readFd, buffer, dtg.getHeader().size);

        if( readBytes == dtg.getHeader().size)
        {
            cout << "Datagram succefully received : " << MessagePrinter::toString(dtg) << endl;
        }
        else
        {
            cout << "Failed to receie payload. nb bytes=" << readBytes << endl;
        }

    }
}
