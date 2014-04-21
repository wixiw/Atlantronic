/*
 * DiscoveryMutex.hpp
 *
 *  Created on: Apr 20, 2014
 *      Author: ard
 */

#ifndef DISCOVERYMUTEX_HPP_
#define DISCOVERYMUTEX_HPP_

#include "linux/tools/robot_interface.h"

namespace arp_stm32
{

class DiscoveryMutex
{
    public:
        DiscoveryMutex();
        ~DiscoveryMutex();

        enum eLockResult
        {
            SUCCEED = 0, FAILED = 1
        };

        //lock is true if the lock failed
        eLockResult lock();
        void unlock();

        static RobotInterface robotItf;
};

} /* namespace arp_stm32 */
#endif /* DISCOVERYMUTEX_HPP_ */
