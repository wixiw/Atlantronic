/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "SimulatedDiscovery.hpp"
#include <ros/package.h>
#include <rtt/Component.hpp>

using namespace arp_stm32;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::SimulatedDiscovery )

SimulatedDiscovery::SimulatedDiscovery(const std::string& name) :
        Discovery(name),
        propStm32ExecutableName("baz_small")
{
}

bool SimulatedDiscovery::configureHook()
{
    if (!MotionScheduler::configureHook())
        return false;

    string atlantronicPath = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    string prog_stm = atlantronicPath + "bin/discovery/" + propStm32ExecutableName;
    string qemu_path = atlantronicPath + "qemu/arm-softmmu/qemu-system-arm";
    int gdb_port = 0;

    int res = m_qemu.init(qemu_path.c_str(), prog_stm.c_str(), gdb_port);
    if (res)
    {
        fprintf(stderr, "qemu_init : error");
        return -1;
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
