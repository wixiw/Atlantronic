/*
 * Timer.hpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_CORE_STATTIMER_HPP_
#define _ARP_CORE_STATTIMER_HPP_

#include "Timer.hpp"
#include <vector>
#include <string>

namespace arp_core
{
    class StatTimer : public Timer
    {
    public :
        StatTimer( int nMax = 20 ) ;
        ~StatTimer() ;

        void Start();
        void Stop();
        void ResetStat();

        void SetMaxBufferSize(const unsigned int s);
        unsigned int GetMaxBufferSize() const;

        std::vector<double> GetRawElapsedTime() const;
        double GetLastElapsedTime() const;
        double GetMeanElapsedTime() const;
        double GetStdDevElapsedTime() const;
        double GetMinElapsedTime() const;
        double GetMaxElapsedTime() const;

        std::vector<double> GetRawRefreshTime() const;
        double GetLastRefreshTime() const;
        double GetMeanRefreshTime() const;
        double GetStdDevRefreshTime() const;
        double GetMinRefreshTime() const;
        double GetMaxRefreshTime() const;

    protected :
        std::vector<double> refreshTimeVector;
        std::vector<double> elapsedTimeVector;
        unsigned int maxBufferSize;
        double lastResetTime;
    } ;
}


#endif /* _ARP_CORE_STATTIMER_HPP_ */
