/*
 * ArdTime.cpp
 *
 *  Created on: Mar 31, 2014
 *      Author: willy
 */

#include "ArdTime.hpp"
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

namespace arp_time
{


ArdAbsoluteTime getAbsoluteTime()
{
    RTT::os::TimeService* ts = RTT::os::TimeService::Instance();
    if( !ts )
    {
        return 0;
    }

    RTT::os::TimeService::ticks now = ts->getTicks();
    ArdAbsoluteTime time = (ArdAbsoluteTime)(ts->nsecs2ticks(now))/1E9;

    return time;
}

ArdTimeDelta getTimeDelta(ArdAbsoluteTime beginTime, ArdAbsoluteTime now)
{
    if( beginTime < 0 || now < 0 || now < beginTime)
    {
        return 0;
    }

    return now - beginTime;
}

ArdAbsoluteTime addTimeAndDelta(ArdAbsoluteTime time, ArdTimeDelta delta)
{
    if( delta < 0 || time < 0 )
    {
        return 0;
    }
    return time + delta;
}


//ArdAbsoluteTime convertTimeFromTimeSpec(const timespec & now)
//{
//    long double time = now.tv_sec + (long double) (now.tv_nsec) / 1E9;
//    return time;
//}
//
//timespec convertTimeSpecFromTime(ArdAbsoluteTime now)
//{
//    timespec time;
//    time.tv_sec = (__time_t)now;
//    time.tv_nsec = (__time_t)((now-time.tv_sec)*1E9);
//    return time;
//}

//Cette fonction fonctionne mais ne devrait plus etre utilisée
//void delta_t(struct timespec *interval, struct timespec begin, struct timespec now)
//{
//    interval->tv_nsec = now.tv_nsec - begin.tv_nsec; /* Subtract 'decimal fraction' first */
//    if (interval->tv_nsec < 0)
//    {
//        interval->tv_nsec += 1000000000; /* Borrow 1sec from 'tv_sec' if subtraction -ve */
//        interval->tv_sec = now.tv_sec - begin.tv_sec - 1; /* Subtract whole number of seconds and return 1 */
//    }
//    else
//    {
//        interval->tv_sec = now.tv_sec - begin.tv_sec; /* Subtract whole number of seconds and return 0 */
//    }
//}
//long double delta_t(struct timespec begin, struct timespec now)
//{
//    timespec delay;
//    delta_t(&delay, begin, now);
//    return timespec2Double(delay);
//}

}
