/*
 * Logger.cpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#include "logger/Logger.hpp"

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

}
}
