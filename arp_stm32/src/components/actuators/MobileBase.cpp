/*
 * MobileBase.cpp
 *
 *  Created on: May 26, 2014
 *      Author: ard
 */

#include "MobileBase.hpp"

using namespace arp_stm32;

MobileBase::MobileBase(const std::string& name) :
        Stm32TaskContext(name)
{
    createOrocosInterface();
}

bool MobileBase::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void Dynamixel::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;
    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    //TODO
    //Update from STM32

    mutex.unlock();
}

void MobileBase::MobileBase()
{
    Stm32TaskContext::updateHook();
}

void MobileBase::createOrocosInterface()
{

}
