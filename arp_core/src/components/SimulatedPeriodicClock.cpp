/*
 * SimulatedPeriodicClock.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#include "SimulatedPeriodicClock.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>
#include <math/core>

using namespace arp_math;
using namespace arp_core;
using namespace arp_time;

ORO_LIST_COMPONENT_TYPE( arp_core::SimulatedPeriodicClock )

SimulatedPeriodicClock::SimulatedPeriodicClock(const std::string name):
           ARDTaskContext(name,ros::package::getPath("arp_core"))
{
    addPort("outClock",outClock);
    addPort("outClockReporting",outClockReporting);
    addPort("outPeriod",outPeriod);
    addPort("outTrigger",outTrigger);
}

bool SimulatedPeriodicClock::startHook()
{
    m_lastTime = 0;
    return true;
}

void SimulatedPeriodicClock::updateHook()
{
    ArdAbsoluteTime now = m_lastTime+getPeriod();
    ArdTimeDelta period = getPeriod();
    m_lastTime = now;

    outClock.write(now);
    outClockReporting.write(now);
    outPeriod.write(period);
    outTrigger.write(0);
}




