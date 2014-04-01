/*
 * ArdTime.hpp
 *
 *  Created on: Mar 31, 2014
 *      Author: willy
 */

#ifndef ARDTIME_HPP_
#define ARDTIME_HPP_

#include <sstream>

namespace arp_time
{

/**
 * Absolute time is in seconds from the Orocos time reference
 */
typedef long double ArdAbsoluteTime;

/**
 * Delta between 2 absolute times in seconds
 */
typedef double ArdTimeDelta;

/**
 * Return the (considered) absolute time compute from Orocos Time Service
 * This is doing a clock request under the hoods
 */
ArdAbsoluteTime getAbsoluteTime();

/**
 * Make the difference between 2 absolute times
 * If parameters are bad we return 0;
 */
ArdTimeDelta getTimeDelta(ArdAbsoluteTime beginTime, ArdAbsoluteTime now);

/**
 * Compute a new absolute time from a time and a duration
 * If parameters are bad we return 0;
 */
ArdAbsoluteTime addTimeAndDelta(ArdAbsoluteTime time, ArdTimeDelta delta);

//--------------------------------------------------------------------
//--- CONVERT

std::string ArdAbsoluteTimeToStr(ArdAbsoluteTime time);

} /* namespace arp_core */
#endif /* ARDTIME_HPP_ */
