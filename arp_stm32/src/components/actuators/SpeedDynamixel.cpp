/*
 * Dynamixel.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/actuators/SpeedDynamixel.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_core;
using namespace std_msgs;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::SpeedDynamixel)

namespace arp_stm32
{

SpeedDynamixel::SpeedDynamixel(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        propId(1),
        propDynamixelFamily(0)
{
    createOrocosInterface();
}

bool SpeedDynamixel::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    if( propDynamixelFamily != DYNAMIXEL_TYPE_AX12 && propDynamixelFamily != DYNAMIXEL_TYPE_RX24 )
    {
        LOG(Error) << "configureHook() : propDynamixelFamily shall be equal to 12 (ax12) or 24 (rx24f) and nothing else. I read type=" << propDynamixelFamily << endlog();
        return false;
    }

    if( propId <= 1 /* not configured */
            || (propDynamixelFamily == DYNAMIXEL_TYPE_AX12 && AX12_MAX_ID <= propId )
            || (propDynamixelFamily == DYNAMIXEL_TYPE_RX24 && RX24_MAX_ID <= propId )
            )
    {
        LOG(Error) << "configureHook() : propId is out of range ]1,max_id]" << endlog();
        return false;
    }

    return true;
}

void SpeedDynamixel::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    attrConnected           = isConnected();
    attrPositionMeasure     = getPosition();

    attrState.stucked         = 0;
    attrState.connected       = attrConnected;
    attrState.target_reached  = 0;
    attrState.position        = attrPositionMeasure;

    mutex.unlock();

    outState.write(attrState);

    std_msgs::Float32 speedCmd;
    if( RTT::NewData == inSpeedCmd.read(speedCmd) )
    {
        attrSpeedCmd = speedCmd.data;
        sendSpeedCmd(attrSpeedCmd);

    }
}

void SpeedDynamixel::sendSpeedCmd(double speed)
{
    int errorCode = m_robotItf.dynamixel_set_speed(propDynamixelFamily, propId, speed);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to set speed to dynamixel (family="<< propDynamixelFamily << ") with ID=" << propId << "." << endlog();
    }
}

bool SpeedDynamixel::isConnected()
{
    bool connected = false;

    switch( propDynamixelFamily )
    {
        case DYNAMIXEL_TYPE_AX12:
            connected = ( m_robotItf.ax12[propId].error.transmit_error == 0);
            break;

        case DYNAMIXEL_TYPE_RX24:
            connected = ( m_robotItf.rx24[propId].error.transmit_error == 0 );
            break;

        default:
            connected = false;
            LOG(Error) << "isConnected() : Unknown family." << endlog();
            break;
    }
    return connected;


}

double SpeedDynamixel::getPosition()
{
    double position = 0.0;

    switch( propDynamixelFamily )
    {
        case DYNAMIXEL_TYPE_AX12:
            position = m_robotItf.ax12[propId].pos;
            break;

        case DYNAMIXEL_TYPE_RX24:
            position = m_robotItf.rx24[propId].pos;
            break;

        default:
            LOG(Error) << "getPosition() : Unknown family." << endlog();
            position = 0.0;
            break;
    }

    return position;
}

void SpeedDynamixel::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addAttribute("attrSpeedCmd",     attrSpeedCmd);
    addAttribute("attrConnected",       attrConnected);
    addAttribute("attrPositionMeasure", attrPositionMeasure);


    addProperty("propId",               propId);
    addProperty("propDynamixelFamily",  propDynamixelFamily);

    addEventPort("inSpeedCmd",       inSpeedCmd);
    addPort("outState",                 outState);
}
}
