/*
 * UbiquityParams.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "UbiquityParams.hpp"
#include <rtt/Component.hpp>

using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_core::UbiquityParams )

UbiquityParams::UbiquityParams(const std::string& name):
        ARDTaskContext(name)
{
}

