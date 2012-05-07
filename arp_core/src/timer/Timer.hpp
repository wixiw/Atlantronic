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
        long double GetTime();
        long double ResetTime();

    private:
        long double t0;
    };
}


#endif /* _ARP_CORE_TIMER_HPP_ */
