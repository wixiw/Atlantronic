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
using namespace arp_msgs;
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
    MatchDataMsg matchData;
    bool color;
    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    color                       = m_robotItf.get_gpio(GPIO_COLOR);
    attrMatchData.start_in      = m_robotItf.get_gpio(GPIO_IN_GO);
    attrMatchData.match_time    = m_robotItf.current_time - m_robotItf.start_time;

    mutex.unlock();

    switch (color)
    {
        case COLOR_RED:
            attrMatchData.color = "red";
            break;
        case COLOR_YELLOW:
        default:
            attrMatchData.color = "yellow";
            break;
    }

    outMatchData.write(attrMatchData);

    std_msgs::Bool readyForMatch;
    if( RTT::NewData == inReadyForMatch.read(readyForMatch) )
    {
        if( readyForMatch.data )
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
    addAttribute("attrStartPlugged", attrMatchData.start_in);
    addAttribute("attrStartColor", attrMatchData.color);
    addAttribute("attrMatchTime", attrMatchData.match_time);

    addPort("outMatchData", outMatchData).doc(
            "Value of match data such as start, color, match time.");

    addEventPort("inReadyForMatch", inReadyForMatch).doc(
            "When set to true, the next start withdraw will be the start signal.");
}
