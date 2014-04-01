/*
 * Timer.hpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_CORE_STATTIMER_HPP_
#define _ARP_CORE_STATTIMER_HPP_

#include "time/ArdTime.hpp"
#include <vector>
#include <string>

namespace arp_core
{
    class StatTimer
    {
    public :
        StatTimer( int nMax = 100 ) ;
        ~StatTimer() ;

        void Start();
        void Stop();
        void ResetStat();

        void SetMaxBufferSize(const unsigned int s);
        unsigned int GetMaxBufferSize() const;

        std::vector<arp_time::ArdTimeDelta> GetRawElapsedTime() const;
        arp_time::ArdTimeDelta GetLastElapsedTime() const;
        arp_time::ArdTimeDelta GetMeanElapsedTime() const;
        arp_time::ArdTimeDelta GetStdDevElapsedTime() const;
        arp_time::ArdTimeDelta GetMinElapsedTime() const;
        arp_time::ArdTimeDelta GetMaxElapsedTime() const;

        std::vector<arp_time::ArdTimeDelta> GetRawRefreshTime() const;
        arp_time::ArdTimeDelta GetLastRefreshTime() const;
        arp_time::ArdTimeDelta GetMeanRefreshTime() const;
        arp_time::ArdTimeDelta GetStdDevRefreshTime() const;
        arp_time::ArdTimeDelta GetMinRefreshTime() const;
        arp_time::ArdTimeDelta GetMaxRefreshTime() const;

        std::string GetReport() const;

        arp_time::ArdAbsoluteTime GetTime();
        arp_time::ArdTimeDelta ResetTime();

    protected :
        std::vector<arp_time::ArdTimeDelta> refreshTimeVector;
        std::vector<arp_time::ArdTimeDelta> elapsedTimeVector;
        unsigned int maxBufferSize;

    private:
        arp_time::ArdAbsoluteTime t0;
    } ;
}


#endif /* _ARP_CORE_STATTIMER_HPP_ */
