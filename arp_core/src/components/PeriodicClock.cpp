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
using namespace arp_time;

ORO_LIST_COMPONENT_TYPE( arp_core::PeriodicClock )

PeriodicClock::PeriodicClock(const std::string name):
           ARDTaskContext(name,ros::package::getPath("arp_core"))
{
    addPort("outClock",outClock);
    addPort("outPeriod",outPeriod);
    addPort("outTrigger",outTrigger);
}

bool PeriodicClock::startHook()
{
    m_lastTime = getAbsoluteTime();
    return true;
}

void PeriodicClock::updateHook()
{
    ArdAbsoluteTime now = getAbsoluteTime();
    ArdTimeDelta period = getTimeDelta(m_lastTime, now);
    m_lastTime = now;

    outClock.write(now);
    outPeriod.write(period);
    outTrigger.write(0);
}




