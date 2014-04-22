/*
 * Tor.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/sensors/Tor.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Tor)

Tor::Tor(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        propInvertSignal(false),
        propTorId(0)
{
    createOrocosInterface();
}

bool Tor::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    if( propTorId <= 0 )
    {
        LOG(Error) << "configureHook : propTorId is out of range, shall be strictly positive." << endlog();
        return false;
    }

    return true;
}

void Tor::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    attrSignal = getGpioValue();
    if( propInvertSignal )
    {
        attrSignal = !attrSignal;
    }

    mutex.unlock();

    outObjectPresent.write(attrSignal);
}

bool Tor::getGpioValue()
{
    return m_robotItf.get_gpio(propTorId);
}

void Tor::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrSignal", attrSignal);

    addProperty("propInvertSignal", propInvertSignal);
    addProperty("propTorId", propTorId);
}

