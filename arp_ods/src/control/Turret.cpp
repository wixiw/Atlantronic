/*
 * Turret.cpp
 *
 *  Created on: 12 février 2012
 *      Author: wla
 */

#include "Turret.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_ods;
using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_ods::Turret)

Turret::Turret(const std::string& name):
    OdsTaskContext(name),

{
    addPort("inSteeringPosition", inSteeringPosition)
        .doc("Input Position command");
    addPort("inDrivingSpeed", inDrivingSpeed)
        .doc("Input Position command");
}

