/*
 * MatchData.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/match_data/MatchData.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
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
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
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

    std_msgs::Bool readyForMatchMsg;
    if( RTT::NewData == inReadyForMatch.read(readyForMatchMsg) )
    {
        attrReadyForMatch = readyForMatchMsg.data;
        if( attrReadyForMatch )
        {
            setReadyForMatch();
        }

    }
}

void MatchData::setReadyForMatch()
{
    int res = m_robotItf.go_enable();

    if (res < 0)
    {
        LOG(Error) << "Failed to enable the start in the stm32 board." << endlog();
    }
    else
    {
        LOG(Info) << "Start enabled in the stm32 board." << endlog();
    }
}

void MatchData::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrStartPlugged", attrStartPlugged);
    addAttribute("attrStartColor", attrStartColor);
    addAttribute("attrReadyForMatch", attrReadyForMatch);


    addPort("outIoStart", outIoStart).doc(
            "Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoStartColor", outIoStartColor).doc("Value of the color switch");

    addEventPort("inReadyForMatch", inReadyForMatch).doc(
            "When set to true, the next start withdraw will be the start signal.");
}