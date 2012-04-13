/*
 * PeriodicClock.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#include "LastComponent.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_master;
using namespace std_msgs;

ORO_LIST_COMPONENT_TYPE( arp_master::LastComponent )

LastComponent::LastComponent(const std::string name):
           MasterTaskContext(name)
{
    addPort("outDeployed",outDeployed);
}

void LastComponent::updateHook()
{
    Bool deployed;
    deployed.data = true;
    outDeployed.write(deployed);
}
