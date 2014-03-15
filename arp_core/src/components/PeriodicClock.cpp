/*
 * PeriodicClock.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#include "PeriodicClock.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>
#include <math/core>

using namespace arp_math;
using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_core::PeriodicClock )

PeriodicClock::PeriodicClock(const std::string name):
           ARDTaskContext(name,ros::package::getPath("arp_core"))
{
    addPort("outClock",outClock);
    addPort("outPeriod",outPeriod);
    addPort("outTrigger",outTrigger);

    clock_gettime(CLOCK_MONOTONIC, &m_absoluteTime);
}

void PeriodicClock::updateHook()
{
    clock_gettime(CLOCK_MONOTONIC, &m_absoluteTime);

    outPeriod.write(getPeriod());
    outClock.write(m_absoluteTime);
    outTrigger.write(0);
}
