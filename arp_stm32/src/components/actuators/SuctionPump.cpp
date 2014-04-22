/*
 * SuctionPump.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/actuators/SuctionPump.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::SuctionPump)

SuctionPump::SuctionPump(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        attrObjectPresent(false),
        propPumpId(-1)
{
    createOrocosInterface();
}

bool SuctionPump::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void SuctionPump::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;
    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }
    attrObjectPresent = getObjectPresent();
    mutex.unlock();

    outObjectPresent.write(attrObjectPresent);

    double suctionPower;
    if( RTT::NewData == inSuctionPowerCmd.read(suctionPower) )
    {
        if( 100 < suctionPower)
        {
            suctionPower = 100;
        }
        if( suctionPower < 0 )
        {
            suctionPower = 0;
        }
        setSuctionPower(suctionPower);
    }
}

void SuctionPump::setSuctionPower(int power)
{
    //TODO JB stm32 ITF
//    int errorCode = m_robotItf.XXX;
//    if (errorCode < 0)
//    {
//        LOG(Error) << "Failed to set suction power on pump with ID=" << propPumpId << "." << endlog();
//        return false;
//    }
//    return true;
}

bool SuctionPump::getObjectPresent()
{
    //TODO JB stm32 ITF
    return false;
}

void SuctionPump::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrObjectPresent", attrObjectPresent);

    addProperty("propPumpId", propPumpId);
}
