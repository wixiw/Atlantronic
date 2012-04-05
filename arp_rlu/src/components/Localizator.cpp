/*
 * Localizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Localizator.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name):
        RluTaskContext(name)
{

}

