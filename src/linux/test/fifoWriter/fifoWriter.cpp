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
#include "ipc/IpcHeader.hpp"
#include <math/math.hpp>
#include "components/usb/DiscoveryIpcTypes.hpp"
#include "components/usb/VersionMessage.hpp"
#include "components/usb/StatusMessage.hpp"
#include "components/usb/LogMessage.hpp"
#include "components/usb/FaultMessage.hpp"
#include "components/usb/EventMessage.hpp"
#include "components/usb/OpponentListMsg.hpp"
#include "components/usb/RawMessage.hpp"

using namespace std;
using namespace arp_math;
using namespace arp_stm32;
using namespace arp_stm32::ipc;

int main()
{
    //Try to open the file, this call is blocking if no reader are present
    int writeFd = ::open("/tmp/wix", O_WRONLY);
    if( writeFd < 0 )
    {
        cout << "Open returned an error : " << strerror(errno) << endl;
        return -1;
    }

    uint8_t buffer[MSG_MAX_SIZE];
    char line[100];
    memset(line, 0, 100);

    IpcHeader header;

    uint8_t data[3];
    data[0] = 1;
    data[1] = 2;
    data[2] = 3;

    do
    {
        if( 0 == strcmp(line, "version"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_VERSION;
            header.size = VersionMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            VersionMessage msg;
            Payload payload;
            msg.setVersion("aa9c07957790bc0c8bab0f1d80d1bad7c4a4452c");
            cout << "Message is : " << msg.toString() << endl;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "status"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_STATUS;
            header.size = StatusMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            StatusMessage msg;
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "log"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_LOG;
            header.size = LogMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            LogMessage msg;
            Payload payload;
            msg.log(LOG_ERROR, "MyFunction", 8, "Bonjour, Bite!");
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "fault"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_FAULT;
            header.size = FaultMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            FaultMessage msg;
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "event"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_EVENT;
            header.size = EventMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            EventMessage msg(EVT_START_MATCH);
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "opplist"))
        {
            OpponentListMsg msg;
            detection_object opp;
            opp.x = 1.3;
            opp.y = 0.7;
            opp.size = 0.2;
            msg.addOpponent(opp);
            opp.x = -0.1;
            opp.y = -0.5;
            opp.size = 0.15;
            msg.addOpponent(opp);

            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_OPP_LIST;
            header.size = 2*sizeof(detection_object);
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "raw"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = 7;
            header.size = 3;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                ::write(writeFd, buffer , HEADER_SIZE);
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            RawMessage msg;
            Payload payload;
            msg.setPayload(data, header.size);
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << uint8ToHexArray(buffer, payload.second) << "." << endl;
                ::write(writeFd, payload.first , payload.second);
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else
        {
            cout << "unknown command" << endl;
        }

        cout << "Choose msg to send (version, status, log, fault, event, opplist or raw)?" << endl;
    } while (std::cin.getline(line, 100));
}


