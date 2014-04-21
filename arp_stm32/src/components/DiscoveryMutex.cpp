/*
 * DiscoveryMutex.cpp
 *
 *  Created on: Apr 20, 2014
 *      Author: ard
 */

#include "DiscoveryMutex.hpp"

using namespace arp_stm32;

RobotInterface DiscoveryMutex::robotItf;

DiscoveryMutex::DiscoveryMutex()
{
}

DiscoveryMutex::~DiscoveryMutex()
{
    pthread_mutex_unlock(&(robotItf.mutex));
}

DiscoveryMutex::eLockResult DiscoveryMutex::lock()
{
    if (pthread_mutex_lock(&(robotItf.mutex)) != 0)
    {
        return FAILED;
    }
    else
    {
        return SUCCEED;
    }
}

void DiscoveryMutex::unlock()
{
    pthread_mutex_unlock(&(robotItf.mutex));
}
;
