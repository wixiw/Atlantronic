/*
 * Timer.cpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 *
 */

#include "Timer.hpp"

using namespace arp_math;
using namespace arp_core;

Timer::Timer()
{
    ResetTime();
}

Timer::~Timer()
{
}

long double Timer::GetTime()
{
    return getTime() - t0;
}

long double Timer::ResetTime()
{
    long double now = getTime();
    long double delay = now - t0;
    t0 = now;
    return delay;
}

