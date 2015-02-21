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

int main()
{
    //Try to open the file, this call is blocking if no reader are present
    int writeFd = ::open("/tmp/carte.in", O_WRONLY);
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
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            VersionMessage msg;
            Payload payload;
            msg.setVersion("aa9c07957790bc0c8bab0f1d80d1bad7c4a4452c");
            cout << "Message is : " << MessagePrinter::toString(msg) << endl;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
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
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
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
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "log"))
        {
        	LogMessage msg;
        	Payload payload;
        	msg.logArd(LOG_ERROR, "MyFunction", 8, "Bonjour, Bite!");

            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_LOG;
            header.size = msg.getSize();
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
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
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
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
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "reboot"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_EVENT;
            header.size = EventMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                cout << "header sent with " << res << " bytes. HEADER_SIZE=" << HEADER_SIZE << endl;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            EventMessage msg(EVT_REBOOT);
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                cout << "payload sent with " << res << " bytes." << endl;
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "list"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_EVENT;
            header.size = EventMessage::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                cout << "header sent with " << res << " bytes. HEADER_SIZE=" << HEADER_SIZE << endl;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            EventMessage msg(EVT_LIST_TASKS);
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                cout << "payload sent with " << res << " bytes." << endl;
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "config"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_CONFIGURATION;
            header.size = ConfigurationMsg::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                cout << "header sent with " << res << " bytes. HEADER_SIZE=" << HEADER_SIZE << endl;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            ConfigurationMsg msg;
            msg.setMatchDuration(5.0);
            msg.setModuleStartConfig( BOOT_ID_CONTROL, true);
            msg.setModuleStartConfig( BOOT_ID_DETECTION, true);
            msg.setModuleStartConfig( BOOT_ID_FAULT, false);//TODO bugged
            msg.setModuleStartConfig( BOOT_ID_DYNAMIXEL, true);
            msg.setModuleStartConfig( BOOT_ID_HOKUYO, true);
            msg.setControlPeriod(200);

            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                cout << "payload sent with " << res << " bytes." << endl;
            }
            else
            {
                cout << "failed to serialize body." << endl;
            }
        }
        else if( 0 == strcmp(line, "x86cmd"))
        {
            memset(buffer, 0, MSG_MAX_SIZE);
            header.magic = MAGIC_NUMBER;
            header.type = MSG_X86_CMD;
            header.size = X86CmdMsg::SIZE;
            if ( header.serialize(buffer) )
            {
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                cout << "header sent with " << res << " bytes. HEADER_SIZE=" << HEADER_SIZE << endl;
            }
            else
            {
                cout << "failed to serialize header." << endl;
            }

            X86CmdMsg msg;
            Payload payload;
            memset(buffer, 0, MSG_MAX_SIZE);
            payload.first = buffer;
            if ( msg.serialize(payload) )
            {
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                cout << "payload sent with " << res << " bytes." << endl;
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
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
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
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
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
                cout << "sending header (" << static_cast<unsigned short>(HEADER_SIZE) << "): " << MessagePrinter::uint8ToHexArray(buffer, HEADER_SIZE) << "." << endl;
                int res = ::write(writeFd, buffer , HEADER_SIZE);
                res ++;
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
                cout << "sending body (" << static_cast<unsigned short>(payload.second) << "): " << MessagePrinter::uint8ToHexArray(buffer, payload.second) << "." << endl;
                int res = ::write(writeFd, payload.first , payload.second);
                res ++;
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

        cout << "***************************************************" << endl;
        cout << "Choose msg to send (version, status, log, fault, list, reboot, config, x86cmd, opplist or raw)?" << endl;
    } while (std::cin.getline(line, 100));
}


