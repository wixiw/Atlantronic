/*
 * Hmi.hpp
 *
 *  Created on: 21 April 2014
 *      Author: jbt
 */

#include "Hmi.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>
#include <ros/package.h>
#include "linux/tools/glplot.h"

using namespace arp_stm32;
using namespace arp_math;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Hmi)

Hmi::Hmi(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
}

bool Hmi::configureHook()
{
    pthread_t tid;
    int res = pthread_create(&tid, NULL, Hmi::task_wrapper, this);
    if (res != 0)
    {
        LOG(Error) << "Failed to spawn HMI thread" << endlog();
        return false;
    }

    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void Hmi::cleanupHook()
{
    m_robotItf.destroy();
    Stm32TaskContext::cleanupHook();
}

void Hmi::updateHook()
{
    Stm32TaskContext::updateHook();

    EstimatedPose2D pose;
    if( inEstimatedPose.read(pose) == NewData )
    {
        VectPlan vp = VectPlan(pose.x(), pose.y(), pose.h());
        int errorCode = m_robotItf.set_position(vp);
        if( errorCode  < 0 )
        {
            LOG(Error) << "Failed to set position." << endlog();
        }

    }

    glplot_update();
}

void* Hmi::task_wrapper(void* arg)
{
    Hmi* hmi = (Hmi*) arg;
    string path = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    glplot_main(path.c_str(), 0, NULL, &hmi->m_robotItf);
    return NULL;
}

void Hmi::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addPort("inEstimatedPose",inEstimatedPose);
}

