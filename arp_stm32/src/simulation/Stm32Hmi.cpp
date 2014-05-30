/*
 * Stm32Hmi.hpp
 *
 *  Created on: 21 April 2014
 *      Author: jbt
 */

#include "Stm32Hmi.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>
#include <ros/package.h>
#include "linux/tools/glplot.h"
#include "SimulatedDiscovery.hpp"

using namespace arp_stm32;
using namespace arp_math;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Stm32Hmi)

Stm32Hmi::Stm32Hmi(const std::string& name) :
Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
}

bool Stm32Hmi::configureHook()
{
    pthread_t tid;
    int res = pthread_create(&tid, NULL, Stm32Hmi::task_wrapper, this);
    if (res != 0)
    {
        LOG(Error) << "Failed to spawn HMI thread" << endlog();
        return false;
    }

    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void Stm32Hmi::cleanupHook()
{
    m_robotItf.destroy();
    Stm32TaskContext::cleanupHook();
}

void Stm32Hmi::updateHook()
{
    Stm32TaskContext::updateHook();
    glplot_update();
}

void* Stm32Hmi::task_wrapper(void* arg)
{
    Stm32Hmi* hmi = (Stm32Hmi*) arg;
    string path = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    glplot_main(path.c_str(), 1, NO_CMD_PROMPT, &SimulatedDiscovery::m_qemu, &hmi->m_robotItf);
    return NULL;
}

void Stm32Hmi::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
}

