/*
 * Odometry.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Odometry.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry )

Odometry::Odometry(const std::string& name):
        RluTaskContext(name)
{

}

