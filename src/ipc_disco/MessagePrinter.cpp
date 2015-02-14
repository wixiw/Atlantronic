/*
 * MessagePrinter.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#include "MessagePrinter.hpp"
#include "DiscoveryIpcTypes.h"
#include <sstream>

using namespace arp_stm32;

using namespace std;


string MessagePrinter::uint8ToHexArray(uint8_t const * const buffer, size_t l)
{
    ostringstream s;
    s << "0x[";
    if( l != 0 )
    {
        s  << hex << static_cast<unsigned int>(buffer[0]);
        for( size_t i=1 ; i<l ; i++)
        {
            s << "|" << hex << static_cast<unsigned int>(buffer[i]);
        }
    }
    s << "]";
    return s.str();
}

string MessagePrinter::charToHexArray(char const * const buffer, size_t l)
{
    ostringstream s;
    s << "0x[";
    if( l != 0 )
    {
        s  << hex << static_cast<unsigned int>(buffer[0]);
        for( size_t i=1 ; i<l ; i++)
        {
            s << "|" << hex << static_cast<unsigned int>(buffer[i]);
        }
    }
    s << "]";
    return s.str();
}

//string RawMessage::toString() const
//{
//    using namespace arp_math;
//    ostringstream s;
//     s << "RawMsg = " << uint8ToHexArray(m_data, MSG_MAX_SIZE) << ".";
//    return s.str();
//}

std::string MessagePrinter::toString(VersionMessage const & msg)
{
    return charToHexArray(msg.getVersion(),VersionMessage::SIZE);
}


std::string MessagePrinter::toString(IpcHeader const & msg)
{
    ostringstream s;
    s << "magic=" << hex << msg.magic << dec << " type=";

    switch(msg.type)
	{
    	case MSG_RESERVED:
    		s << "MSG_RESERVED";
    		break;
    	case MSG_VERSION:
    		s << "MSG_VERSION";
    		break;
    	case MSG_STATUS:
    		s << "MSG_STATUS";
    		break;
    	case MSG_LOG:
    		s << "MSG_LOG";
    		break;
    	case MSG_FAULT:
    		s << "MSG_FAULT";
    		break;
    	case MSG_EVENT:
    		s << "MSG_EVENT";
    		break;
    	case MSG_OPP_LIST:
    		s << "MSG_OPP_LIST";
    		break;
    	case MSG_HOKUYO_SCAN:
    		s << "MSG_HOKUYO_SCAN";
    		break;
    	case MSG_X86_CMD:
    		s << "MSG_X86_CMD";
    		break;
    	case MSG_CONFIGURATION:
    		s << "MSG_X86_CMD";
    		break;
    	case MSG_GYRO_CMD:
    		s << "MSG_GYRO_CMD";
    		break;

    	default:
    		s << "Unknown message type";
    		break;
	}

    s << " l=" << static_cast<unsigned int>(msg.size);
    return s.str();
}

std::string MessagePrinter::toString(Datagram const & msg)
{
	return MessagePrinter::toString(msg.getHeader()) + " " + uint8ToHexArray(msg.getPayload().first,MSG_MAX_SIZE);
}

string MessagePrinter::toString(LogMessage const & msg)
{
	return msg.getLogText();
}
//
//string OpponentListMsg::toString() const
//{
//    ostringstream os;
//    os << "OppList(" << m_oppList.size() << ")[x,y,size] = [";
//    std::list<detection_object>::const_iterator i;
//    detection_object obj;
//    for( i = m_oppList.begin() ; i != m_oppList.end() ; i++)
//    {
//        obj = *i;
//        os << obj.x << "," << obj.y << ", " << obj.size << "|";
//    }
//    os << "]";
//    return os.str();
//}
