/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */
#include "SimulatedDiscovery.hpp"
#include <rtt/Component.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>

using namespace arp_stm32;
using namespace std;
using namespace RTT;

#define LOG(level) RTT::log(level)<<"["<<getName()<<"] "

ORO_LIST_COMPONENT_TYPE( arp_stm32::SimulatedDiscovery )

SimulatedDiscovery::SimulatedDiscovery(const std::string& name) :
        Discovery(name),
        propStm32ExecutableName("baz_small")
{
}

SimulatedDiscovery::~SimulatedDiscovery()
{
    m_qemu.destroy();
}



bool SimulatedDiscovery::configureHook()
{
    if (!MotionScheduler::configureHook())
        return false;

    string atlantronicPath = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    string prog_stm = atlantronicPath + "bin/discovery/" + propStm32ExecutableName;
    string qemu_path = atlantronicPath + "qemu/arm-softmmu/qemu-system-arm";
    int gdb_port = 0;

    if( boost::filesystem::exists(qemu_path.c_str()) == false )
    {
        LOG(Error) << "configureHook() : qemu exec was not found." << endlog();
        return false;
    }

    if( boost::filesystem::exists(prog_stm.c_str()) == false )
    {
        LOG(Error) << "configureHook() : stm32 binary was not found." << endlog();
        return false;
    }

    int res = m_qemu.init(qemu_path.c_str(), prog_stm.c_str(), gdb_port);
    if (res)
    {
        //TODO LOG
        //qemu_init : error
        return false;
    }

    res = m_robotItf.init("discovery", m_qemu.file_board_read, m_qemu.file_board_write, robotItfCallbackWrapper, this);
    if (res != 0)
    {
        return false;
    }

    return true;
}

void SimulatedDiscovery::createOrocosInterface()
{
    Discovery::createOrocosInterface();

    addProperty("propStm32ExecutableName", propStm32ExecutableName);
}
