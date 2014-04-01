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

std::string ArdAbsoluteTimeToStr(ArdAbsoluteTime time)
{
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(3);
    ss << time;
    std::string s = ss.str();
    if(s[s.find_last_not_of('0')] == '.') {
        s.erase(s.size() - 4);
    }
    return s;
}

}
