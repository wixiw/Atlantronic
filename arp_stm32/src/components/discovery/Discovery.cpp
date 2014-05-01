/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_core;
using namespace arp_msgs;
using namespace arp_stm32;
using namespace std;
using namespace RTT;

#define LOG(level) RTT::log(level)<<"["<<getName()<<"] "

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

Discovery::Discovery(const std::string& name) :
        MotionScheduler(name, "arp_stm32"),
        m_robotItf(DiscoveryMutex::robotItf),
        propDeviceName("/dev/discovery")
{
    createOrocosInterface();
    createRosInterface();
}

Discovery::~Discovery()
{
    m_robotItf.destroy();
}

bool Discovery::configureHook()
{
    if (!MotionScheduler::configureHook())
        return false;

    int res = m_robotItf.init("discovery", propDeviceName.c_str(), propDeviceName.c_str(), robotItfCallbackWrapper, this);
    if (res != 0)
    {
        LOG(Error) << "configureHook() : failed to init robot_itf with error code : " << res << endlog();
        return false;
    }

    return true;
}

void Discovery::cleanupHook()
{
    m_robotItf.destroy();
    MotionScheduler::cleanupHook();
}

void Discovery::updateHook()
{
    MotionScheduler::updateHook();
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;

    //The period is null if the component is a slave or on file activity descriptor
    //if the component is periodic we should not call the updateHook
    if( discovery->getPeriod() == 0 )
    {
        discovery->updateHook();
    }
}

bool Discovery::ooReset()
{
    int res = m_robotItf.reboot();

    if (res < 0)
    {
        LOG(Error) << "ooReset() : failed to reboot with error code : " << res << endlog();
        return false;
    }
    return true;
}

bool Discovery::srvResetStm32(EmptyWithSuccessSrv::Request& req, EmptyWithSuccessSrv::Response& res)
{
    res.success = ooReset();
    return res.success;
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addProperty("propDeviceName", propDeviceName);

    addOperation("ooReset", &Discovery::ooReset, this, OwnThread).doc("Reset the stm32 board.");
}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
    nh.advertiseService("/MatchData/resetStm32", &Discovery::srvResetStm32, this);
}
