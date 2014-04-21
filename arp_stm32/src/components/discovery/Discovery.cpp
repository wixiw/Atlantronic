/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

Discovery::Discovery(const std::string& name) :
        MotionScheduler(name, "arp_stm32"),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
    createRosInterface();
}

bool Discovery::configureHook()
{
    int res = m_robotItf.init("discovery", "/dev/discovery", "/dev/discovery", robotItfCallbackWrapper, this);
    if (res != 0)
    {
        return false;
    }

    if (!MotionScheduler::configureHook())
        return false;

    return true;
}

void Discovery::cleanupHook()
{
    m_robotItf.destroy();
    MotionScheduler::cleanupHook();
}

void Discovery::updateHook()
{
    MotionScheduler::updateHook();
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->updateHook();
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
}
