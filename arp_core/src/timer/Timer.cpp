/*
 * Timer.cpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 *
 */

#include "Timer.hpp"
#include <ros/ros.h>

using namespace arp_core;

Timer::Timer()
{
    ResetTime();
}

Timer::~Timer()
{
}

double Timer::GetTime()
{
    return ros::WallTime::now().toSec() - t0;
}

void Timer::ResetTime()
{
    t0 = ros::WallTime::now().toSec();
}

