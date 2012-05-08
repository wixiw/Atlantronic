/*
 * Logger.cpp
 *
 *  Created on: 5 mars 2012
 *      Author: Boris
 */

#include "LSL/Logger.hpp"

#include <iostream>
#include <ostream>

using namespace std;

namespace arp_rlu { namespace lsl {


// Global static pointer used to ensure a single instance of the class.
arp_core::log::Logger* Logger::m_pInstance = NULL;
arp_core::log::Category* Logger::m_pCat = NULL;

arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority)
{
    return arp_rlu::lsl::Logger::Category() << priority;
}


}}
