/*
 * StatTimer.cpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 */

#include "StatTimer.hpp"

#include <numeric>
#include <algorithm>
#include <iostream>
#include <sstream>

using namespace arp_core;
using namespace arp_time;

StatTimer::StatTimer( int nMax ) :
maxBufferSize( nMax )
{
    ResetTime();
    ResetStat();
}

StatTimer::~StatTimer()
{
    ResetStat();
}

void StatTimer::Start()
{
    ArdTimeDelta dt =  this->ResetTime();
    refreshTimeVector.push_back( dt );
    while( refreshTimeVector.size() > maxBufferSize )
        refreshTimeVector.erase( refreshTimeVector.begin() );
}

void StatTimer::Stop()
{
    ArdTimeDelta dt = this->GetTime();
    elapsedTimeVector.push_back( dt );
    while( elapsedTimeVector.size() > maxBufferSize )
        elapsedTimeVector.erase( elapsedTimeVector.begin() );
}

void StatTimer::ResetStat()
{
    refreshTimeVector.clear();
    elapsedTimeVector.clear();
}

void StatTimer::SetMaxBufferSize(const unsigned int s)
{
    maxBufferSize = s;
    while( elapsedTimeVector.size() > maxBufferSize )
        elapsedTimeVector.erase( elapsedTimeVector.begin() );
    while( refreshTimeVector.size() > maxBufferSize )
        refreshTimeVector.erase( refreshTimeVector.begin() );
}

unsigned int StatTimer::GetMaxBufferSize() const
{
    return maxBufferSize;
}

std::vector<ArdTimeDelta> StatTimer::GetRawElapsedTime() const
{
    return elapsedTimeVector;
}

ArdTimeDelta StatTimer::GetLastElapsedTime() const
{
    if( elapsedTimeVector.empty() )
        return 0.0;
    else
        return elapsedTimeVector.back();
}

ArdTimeDelta StatTimer::GetMeanElapsedTime() const
{
    if( elapsedTimeVector.empty() )
        return 0.0;

    double sum = std::accumulate(elapsedTimeVector.begin(), elapsedTimeVector.end(), 0.0 );
    return sum / (double)elapsedTimeVector.size();
}

ArdTimeDelta StatTimer::GetStdDevElapsedTime() const
{
    if( elapsedTimeVector.empty() )
        return 0.0;

    double mean = this->GetMeanElapsedTime();
    std::vector<ArdTimeDelta> diff;
    for(std::vector<ArdTimeDelta>::const_iterator it = elapsedTimeVector.begin(); it != elapsedTimeVector.end(); ++it)
        diff.push_back( (*it) - mean );

    return std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / (double)elapsedTimeVector.size();
}

ArdTimeDelta StatTimer::GetMinElapsedTime() const
{
    if( elapsedTimeVector.empty() )
        return 0.0;
    return *(std::min_element(elapsedTimeVector.begin(), elapsedTimeVector.end() ));
}

ArdTimeDelta StatTimer::GetMaxElapsedTime() const
{
    if( elapsedTimeVector.empty() )
        return 0.0;
    return *(std::max_element(elapsedTimeVector.begin(), elapsedTimeVector.end() ));
}

std::vector<ArdTimeDelta> StatTimer::GetRawRefreshTime() const
{
    return refreshTimeVector;
}

ArdTimeDelta StatTimer::GetLastRefreshTime() const
{
    if( refreshTimeVector.empty() )
        return 0.0;
    else
        return refreshTimeVector.back();
}

ArdTimeDelta StatTimer::GetMeanRefreshTime() const
{
    if( refreshTimeVector.empty() )
        return 0.0;

    ArdTimeDelta sum = std::accumulate(refreshTimeVector.begin(), refreshTimeVector.end(), 0.0 );
    return sum / (double)refreshTimeVector.size();
}

ArdTimeDelta StatTimer::GetStdDevRefreshTime() const
{
    if( refreshTimeVector.empty() )
        return 0.0;

    ArdTimeDelta mean = this->GetMeanRefreshTime();
    std::vector<ArdTimeDelta> diff;
    for(std::vector<ArdTimeDelta>::const_iterator it = refreshTimeVector.begin(); it != refreshTimeVector.end(); ++it)
        diff.push_back( (*it) - mean );

    return std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / (double)refreshTimeVector.size();
}

ArdTimeDelta StatTimer::GetMinRefreshTime() const
{
    if( refreshTimeVector.empty() )
        return 0.0;
    return *(std::min_element(refreshTimeVector.begin(), refreshTimeVector.end() ));
}

ArdTimeDelta StatTimer::GetMaxRefreshTime() const
{
    if( refreshTimeVector.empty() )
        return 0.0;
    return *(std::max_element(refreshTimeVector.begin(), refreshTimeVector.end() ));
}

std::string StatTimer::GetReport() const
{
    std::stringstream info;
    info << "==============================================" << std::endl;
    info << " Performance Report (ms)" << std::endl;
    info << "----------------------------------------------" << std::endl;
    info << "  [*] Number of samples used : " << GetRawRefreshTime().size() << std::endl;
    info << "  [*] Actual loop period   : mean=" << GetMeanRefreshTime() * 1000.0;
    info << "  , stddev=" << GetStdDevRefreshTime() * 1000.0;
    info << "  , min=" << GetMinRefreshTime() * 1000.0;
    info << "  , max=" << GetMaxRefreshTime() * 1000.0;
    info << "  , last=" << GetLastRefreshTime() * 1000.0 << std::endl;
    /*info << "  [*] Raw actual loop periods :  ( ";
     for(std::vector<long double>::const_iterator it = GetRawRefreshTime().begin(); it != GetRawRefreshTime().end(); ++it)
     info << (*it) * 1000.0 << " ";
     info << " )" << std::endl;*/
    info << "  [*] Loop duration    : mean=" << GetMeanElapsedTime() * 1000.0;
    info << "  , stddev=" << GetStdDevElapsedTime() * 1000.0;
    info << "  , min=" << GetMinElapsedTime() * 1000.0;
    info << "  , max=" << GetMaxElapsedTime() * 1000.0;
    info << "  , last=" << GetLastElapsedTime() * 1000.0 << std::endl;
    /*info << "  [*] Raw loop durations :  ( ";
     for(std::vector<long double>::const_iterator it = GetRawElapsedTime().begin(); it != GetRawElapsedTime().end(); ++it)
     info << (*it) * 1000.0 << " ";
     info << " )" << std::endl; */

    return info.str();
}

ArdAbsoluteTime StatTimer::GetTime()
{
    return getAbsoluteTime() - t0;
}

ArdTimeDelta StatTimer::ResetTime()
{
    ArdAbsoluteTime now = getAbsoluteTime();
    ArdTimeDelta delay = now - t0;
    t0 = now;
    return delay;
}
