/*
 * Logger.cpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#include "models/Logger.hpp"

#include <iostream>
#include <ostream>

using namespace std;

namespace arp_model {

// Global static pointer used to ensure a single instance of the class.
arp_core::log::Logger* Logger::m_pInstance = NULL;
arp_core::log::Category* Logger::m_pCat = NULL;

arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority)
{
    return arp_model::Logger::Category() << priority;
}


}
