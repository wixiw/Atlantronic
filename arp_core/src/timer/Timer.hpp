/*
 * Timer.hpp
 *
 *  Created on: 25 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_CORE_TIMER_HPP_
#define _ARP_CORE_TIMER_HPP_

namespace arp_core
{
    class Timer
    {
    public:
        Timer();
        ~Timer();
        double GetTime();
        void ResetTime();

    private:
        double t0;
    };
}


#endif /* _ARP_CORE_TIMER_HPP_ */
