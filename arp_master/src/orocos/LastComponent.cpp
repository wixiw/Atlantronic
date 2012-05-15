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

LastComponent::~LastComponent()
{
    system("sh /opt/ard/arp_core/script/linux/clear_files.sh");
}

bool LastComponent::startHook()
{
    Bool deployed;
    deployed.data = true;
    outDeployed.write(deployed);

    system("sh /opt/ard/arp_core/script/linux/deployment_deployed.sh");
    return true;
}
