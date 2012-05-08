/*
 * Logger.cpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#include "LoggerSystem.hpp"

#define stringify( name ) # name

namespace arp_core
{
namespace log
{

const char * Category::priorityLevelNames[10] = {
        stringify( EMERG ),
        stringify( FATAL ),
        stringify( ALERT ),
        stringify( CRIT ),
        stringify( ERROR ),
        stringify( WARN ),
        stringify( NOTICE ),
        stringify( INFO ),
        stringify( DEBUG ),
        stringify( NOTSET )
    };

const char * ColorStreamCategory::priorityLevelColors[10] = {
        "\033[1;31m",  // EMERG
        "\033[1;31m",  // FATAL
        "\033[1;31m",  // ALERT
        "\033[1;31m",  // CRIT
        "\033[1;31m",  // ERROR
        "\033[1;33m",  // WARN
        "\033[1;35m",  // NOTICE
        "\033[1;34m",  // NOTICE
        "\033[0m",  // DEBUG
        "\033[0m"   // NOTSET
    };

}
}
