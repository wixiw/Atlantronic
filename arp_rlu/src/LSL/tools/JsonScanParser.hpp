/*
 * JsonScanParser.hpp
 *
 *  Created on: 2 mars 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_
#define _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_

#include <tools/vjson/JsonDocument.hpp>

#include "LSL/LaserScan.hpp"

namespace arp_rlu { namespace lsl {

class JsonScanParser : vjson::JsonDocument
{
    public:
        JsonScanParser();
        JsonScanParser(std::string filename);

        bool parse(const char *filename);
        bool getScan(LaserScan & ls);
};

}}


#endif // _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_
