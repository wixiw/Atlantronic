/*
 * MatchData.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "MatchData.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::MatchData)

MatchData::MatchData(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        attrStartPlugged(false)
{
    createOrocosInterface();
    createRosInterface();
}

bool MatchData::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void MatchData::updateHook()
{
    Stm32TaskContext::updateHook();
    Start start;
    StartColor color;

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    control_usb_data* last_data = &(m_robotItf.last_control_usb_data);

    attrStartPlugged = m_robotItf.get_gpio(GPIO_IN_GO);
    attrStartColor = m_robotItf.get_gpio(GPIO_COLOR);

    mutex.unlock();

    start.go = attrStartPlugged;
    outIoStart.write(start);

    switch (attrStartColor)
    {
        case COLOR_RED:
            color.color = "red";
            break;
        case COLOR_YELLOW:
        default:
            color.color = "yellow";
            break;
    }
    outIoStartColor.write(color);
}

bool MatchData::ooReset()
{
    int res = m_robotItf.reboot();

    if (res < 0)
    {
        LOG(Error) << "Failed to reset the stm32 board." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Resetting the stm32 board." << endlog();
        return true;
    }
}

bool MatchData::ooEnableStart()
{
    int res = m_robotItf.go_enable();

    if (res < 0)
    {
        LOG(Error) << "Failed to enable the start in the stm32 board." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Start enabled in the stm32 board." << endlog();
        return true;
    }
}

bool MatchData::srvResetStm32(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooReset();
    return res.success;
}

bool MatchData::srvEnableStart(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooEnableStart();
    return res.success;
}

void MatchData::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrStartPlugged", attrStartPlugged);
    addAttribute("attrStartColor", attrStartColor);

    addPort("outIoStart", outIoStart).doc(
            "Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoStartColor", outIoStartColor).doc("Value of the color switch");

    addOperation("ooReset", &MatchData::ooReset, this, OwnThread).doc("Reset the stm32 board.");
    addOperation("ooEnableStart", &MatchData::ooEnableStart, this, OwnThread).doc(
            "Informs the stm32 that the next start withdraw sill be the match begining.");
}

void MatchData::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvResetStm32 = nh.advertiseService("/MatchData/resetStm32", &MatchData::srvResetStm32, this);
    m_srvEnableStart = nh.advertiseService("/MatchData/enableStart", &MatchData::srvEnableStart, this);
}
