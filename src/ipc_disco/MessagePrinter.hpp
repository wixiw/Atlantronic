/*
 * MessagePrinter.hpp
 *
 *  Created on: Dec 29, 2014
 *      Author: willy
 */

#ifndef MESSAGEPRINTER_HPP_
#define MESSAGEPRINTER_HPP_

#include "ipc/IpcMsg.hpp"
#include "ipc/IpcHeader.hpp"
#include "ipc/Datagram.hpp"
#include "ipc_disco/RawMessage.hpp"
#include "ipc_disco/VersionMessage.hpp"
#include "ipc_disco/LogMessage.hpp"

namespace arp_stm32
{

class MessagePrinter
{
    public:
        static std::string toString(VersionMessage const & msg);
        static std::string toString(IpcHeader const & msg);
        static std::string toString(Datagram const & msg);
        static std::string toString(LogMessage const & msg);

        /**
         * Build a char buffer as an array of hex numbers 0x[??|??|...]
         */
        static std::string uint8ToHexArray(uint8_t const * const buffer, size_t l);
        static std::string charToHexArray(char const * const buffer, size_t l);

};

} /* namespace arp_stm32 */
#endif


