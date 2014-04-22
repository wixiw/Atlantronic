/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>
#include <ros/package.h>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

Discovery::Discovery(const std::string& name) :
        MotionScheduler(name, "arp_stm32"),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
    createRosInterface();
}

bool Discovery::configureHook()
{
    int simu = 0; // TODO 0 ou 1 selon deploiement
    string atlantronicPath = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    string prog_stm = atlantronicPath + "bin/discovery/baz";
    string qemu_path = atlantronicPath + "qemu/arm-softmmu/qemu-system-arm";
    int gdb_port = 0;
    const char* file_stm_read = "/dev/discovery";
    const char* file_stm_write = "/dev/discovery";

    if(simu)
    {
        int res = qemu.init(qemu_path.c_str(), prog_stm.c_str(), gdb_port);
        if( res )
        {
            fprintf(stderr, "qemu_init : error");
            return -1;
        }

        file_stm_read = qemu.file_board_read;
        file_stm_write = qemu.file_board_write;
    }

    int res = m_robotItf.init("discovery", file_stm_read, file_stm_write, robotItfCallbackWrapper, this);
    if (res != 0)
    {
        return false;
    }

    if (!MotionScheduler::configureHook())
        return false;

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
    discovery->updateHook();
}

bool Discovery::ooReset()
{
    int res = m_robotItf.reboot();

    if (res < 0)
    {
        return false;
    }
    return true;
}

bool Discovery::srvResetStm32(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooReset();
    return res.success;
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addOperation("ooReset", &Discovery::ooReset, this, OwnThread).doc("Reset the stm32 board.");
}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
    nh.advertiseService("/MatchData/resetStm32", &Discovery::srvResetStm32, this);
}
