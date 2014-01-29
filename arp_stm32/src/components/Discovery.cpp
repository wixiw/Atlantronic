/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include <rtt/Component.hpp>

using namespace arp_stm32;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

Discovery::Discovery(const std::string& name) :
        Stm32TaskContext(name)
{
    createOrocosInterface();
}

bool Discovery::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    m_robotItf.init("discovery", "/dev/discovery0", "/dev/discovery0", robotItfCallbackWrapper, this);

    return true;
}

void Discovery::updateHook()
{
    Stm32TaskContext::updateHook();
}

void Discovery::createOrocosInterface()
{
    addAttribute("time", m_robotItf.current_time);
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->robotItfUpdated();
}

void Discovery::robotItfUpdated()
{

}
