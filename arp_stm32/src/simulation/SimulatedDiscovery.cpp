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
using namespace std_msgs;
using namespace std;
using namespace RTT;

#define LOG(level) RTT::log(level)<<"["<<getName()<<"] "

ORO_LIST_COMPONENT_TYPE( arp_stm32::SimulatedDiscovery )

Qemu SimulatedDiscovery::m_qemu;

SimulatedDiscovery::SimulatedDiscovery(const std::string& name) :
        Discovery(name),
        propStm32ExecutableName("baz_small")
{
    createOrocosInterface();
}

SimulatedDiscovery::~SimulatedDiscovery()
{
    m_qemu.destroy();
}



bool SimulatedDiscovery::configureHook()
{
    string atlantronicPath = ros::package::getPath("arp_stm32") + "/src/Atlantronic/";
    string prog_stm = atlantronicPath + "bin/discovery/" + propStm32ExecutableName;
    //string prog_stm = ros::package::getPath("arp_stm32") + "/bin/discovery/" + propStm32ExecutableName;
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
        LOG(Error) << "configureHook() : qemu.init() failed" << endlog();
        return false;
    }

    propDeviceName = m_qemu.file_board_read;
    res = m_robotItf.init("discovery", m_qemu.file_board_read, m_qemu.file_board_write, NULL, robotItfCallbackWrapper, this);
    if (res != 0)
    {
        return false;
    }

    return true;
}

void SimulatedDiscovery::updateHook()
{
    Bool io;

    if(NewData == inFakeUserButton1.read(io) )
    {
        LOG(Info) << "updateHook() : user button1 toggle requested." << endlog();
        m_qemu.set_io(GPIO_IN_BTN1, io.data);
        io.data = false;
        m_qemu.set_io(GPIO_IN_BTN1, io.data);
    }

    if(NewData == inFakeUserButton2.read(io) )
    {
        LOG(Info) << "updateHook() : user button2 toggle requested." << endlog();
        m_qemu.set_io(GPIO_IN_BTN2, io.data);
        io.data = false;
        m_qemu.set_io(GPIO_IN_BTN2, io.data);
    }

    if(NewData == inFakeStart.read(io) )
    {
        LOG(Info) << "updateHook() : start value changed newvalue = " << (int)io.data << "." << endlog();
        attrFakeStart = io.data;
        m_qemu.set_io(GPIO_IN_GO, io.data);
    }

    Discovery::updateHook();
}


bool SimulatedDiscovery::breakUpdateHook()
{
    m_qemu.destroy();
    return true;
}

bool SimulatedDiscovery::ooReset()
{
    return true;
}

void SimulatedDiscovery::createOrocosInterface()
{
    addProperty("propStm32ExecutableName", propStm32ExecutableName);

    addAttribute("attrFakeStart", attrFakeStart);

    addPort("inFakeUserButton1", inFakeUserButton1);
    addPort("inFakeUserButton2", inFakeUserButton2);
    addPort("inFakeStart", inFakeStart);
}
