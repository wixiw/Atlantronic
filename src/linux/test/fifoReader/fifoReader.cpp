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
#include "ipc/RawMessage.hpp"
#include <math/math.hpp>
#include "ipc/Datagram.hpp"
#include "components/usb/DiscoveryIpcTypes.hpp"
#include "components/usb/VersionMessage.hpp"
#include "components/usb/StatusMessage.hpp"
#include "components/usb/LogMessage.hpp"
#include "components/usb/FaultMessage.hpp"
#include "components/usb/EventMessage.hpp"
#include "components/usb/OpponentListMsg.hpp"

using namespace std;
using namespace arp_math;
using namespace arp_stm32;
using namespace arp_stm32::ipc;

int main()
{
    //Try to open the file, this call is blocking if no reader are present
    int readFd = ::open("/tmp/wix2", O_RDONLY);
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
            cout << "Datagram succefully received : " << dtg.toString() << endl;
        }
        else
        {
            cout << "Failed to receie payload. nb bytes=" << readBytes << endl;
        }

    }
}
