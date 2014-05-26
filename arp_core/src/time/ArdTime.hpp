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
 * Create a while loop to wait a function to answer true until a timeout explodes
 * YOU MUST provide in your code a "double chrono=0.0;" variable !
 * @param function : a boolean function that returns true when the loop must continue
 * @param timeout : a maximal time in s
 * @param sleep : sleeping delay each time function is true in s
 */
#define whileTimeout(function, timeout, sleep ) \
        while( function && chrono >= 0 && chrono < timeout )       \
        {                                           \
            chrono += sleep;                        \
            usleep(sleep*1E6);                      \
        }

/**
 * Use this define after a whileTimeout to check
 * if the timeout has expired
 * use it like a if else case :
 * IfWhileTimeoutExpired
 * {
 *    LOG(Error) << "Timeout over !" << endlog();
 * }
 * else
 * {
 *    LOG(Info) << "Great ! it worked." << endlog();
 * }
 */
#define IfWhileTimeoutExpired(timeout)  if( chrono >= timeout )


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
