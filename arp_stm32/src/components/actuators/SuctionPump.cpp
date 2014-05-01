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
using namespace std_msgs;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::SuctionPump)

SuctionPump::SuctionPump(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        propPumpId(-1)
{
    createOrocosInterface();
}

bool SuctionPump::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    if( propPumpId < 0 || PUMP_MAX <= propPumpId )
    {
        LOG(Error) << "configureHook() : propPumpId is out of range [0;" << (int) PUMP_MAX << "]." << endlog();
        return false;
    }

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
    attrObjectPresent.data = getObjectPresent();
    mutex.unlock();

    outObjectPresent.write(attrObjectPresent);

    UInt8 suctionPower;
    if( RTT::NewData == inSuctionPowerCmd.read(suctionPower) )
    {
        if( 100 < suctionPower.data)
        {
            suctionPower.data = 100;
        }
        setSuctionPower(suctionPower);
        attrPumpCommand = suctionPower.data;
    }
}

void SuctionPump::setSuctionPower(UInt8 power)
{
    int errorCode = m_robotItf.pump(propPumpId, power.data);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to set suction power on pump with ID=" << propPumpId << "." << endlog();
    }
}

bool SuctionPump::getObjectPresent()
{
    return m_robotItf.pump_is_blocked(propPumpId);
}

void SuctionPump::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addAttribute("attrObjectPresent", attrObjectPresent.data);
    addAttribute("attrObjectPresent", attrPumpCommand);

    addProperty("propPumpId", propPumpId);

    addPort("outObjectPresent", outObjectPresent);
    addEventPort("inSuctionPowerCmd", inSuctionPowerCmd);
}
