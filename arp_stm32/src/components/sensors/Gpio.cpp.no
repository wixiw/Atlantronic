/*
 * Gpio.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "Gpio.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Gpio)

Gpio::Gpio(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
    createRosInterface();
}

bool Gpio::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void Gpio::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    control_usb_data* last_data = &(m_robotItf.last_control_usb_data);

    mutex.unlock();
}

void Gpio::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
}

void Gpio::createRosInterface()
{
    ros::NodeHandle nh;
}
