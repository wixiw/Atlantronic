/*
 * Dynamixel.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/actuators/Dynamixel.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_core;
using namespace std_msgs;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Dynamixel)

namespace arp_stm32
{

Dynamixel::Dynamixel(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        propPrecision(0.17),
        propMaxTorque(20),
        propId(1),
        propDynamixelFamily(0)
{
    createOrocosInterface();
}

bool Dynamixel::configureHook()
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

    sendMaxTorqueCmd(propMaxTorque);
    sendPrecisionCmd(propPrecision);

    return true;
}

void Dynamixel::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    attrState.connected       = isConnected();
    attrState.stucked         = isStucked();
    attrState.target_reached   = isTargetReached();
    attrState.position       = getPosition();

    mutex.unlock();

    outState.write(attrState);

    std_msgs::Float32 positionCmd;
    if( RTT::NewData == inPositionCmd.read(positionCmd) )
    {
        attrPositionCmd = positionCmd.data;
        sendPositionCmd(attrPositionCmd);

    }

    std_msgs::UInt8 torqueCmd;
    if( RTT::NewData == inMaxTorqueAllowed.read(torqueCmd) )
    {
        attrMaxTorque = torqueCmd.data;
        if( 100 < attrMaxTorque)
        {
            attrMaxTorque = 100;
        }
        if( attrMaxTorque < 0)
        {
            attrMaxTorque = 0;
        }
        sendMaxTorqueCmd(attrMaxTorque);
    }
}

void Dynamixel::sendPositionCmd(double position)
{
    int errorCode = m_robotItf.dynamixel_set_goal_position(propDynamixelFamily, propId, position);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel with ID=" << propId << "." << endlog();
    }
}


void Dynamixel::sendMaxTorqueCmd(int percentage)
{
    int errorCode = m_robotItf.dynamixel_set_max_torque(propDynamixelFamily, propId, percentage);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel RX24F with ID=" << propId << "." << endlog();
    }
}

void Dynamixel::sendPrecisionCmd(double precision)
{
    int errorCode = m_robotItf.dynamixel_set_target_reached_threshold(propDynamixelFamily, propId, precision);
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to set precision to dynamixel RX24F with ID=" << propId << "." << endlog();
    }
}

bool Dynamixel::isTargetReached()
{
    bool targetReached = false;

    //TODO JB stm32 ITF
    //faire mieux => à passer dans robot itf please

    switch( propDynamixelFamily )
    {
        case DYNAMIXEL_TYPE_AX12:
            targetReached = ( (m_robotItf.ax12[propId].flags & DYNAMIXEL_FLAG_TARGET_REACHED ) != 0 );
            break;

        case DYNAMIXEL_TYPE_RX24:
            targetReached = ( (m_robotItf.rx24[propId].flags & DYNAMIXEL_FLAG_TARGET_REACHED ) != 0 );
            break;

        default:
            targetReached = false;
            break;
    }

    return targetReached;
}

bool Dynamixel::isStucked()
{
    bool stuck = false;
    switch( propDynamixelFamily )
    {
        case DYNAMIXEL_TYPE_AX12:
            stuck = ( (m_robotItf.ax12[propId].flags & DYNAMIXEL_FLAG_STUCK ) != 0 );
            break;

        case DYNAMIXEL_TYPE_RX24:
            stuck = ( (m_robotItf.rx24[propId].flags & DYNAMIXEL_FLAG_STUCK ) != 0 );
            break;

        default:
            stuck = false;
            break;
    }
    return stuck;
}

bool Dynamixel::isConnected()
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
            break;
    }
    return connected;


}

double Dynamixel::getPosition()
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
            position = 0.0;
            break;
    }

    return position;
}

void Dynamixel::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addAttribute("attrConnected",       attrState.connected);
    addAttribute("attrStucked",         attrState.stucked);
    addAttribute("attrTargetReached",   attrState.target_reached);
    addAttribute("attrPosition",        attrState.position);
    addAttribute("attrPositionCmd",     attrPositionCmd);
    addAttribute("attrMaxTorque",       attrMaxTorque);


    addProperty("propPrecision", propPrecision);
    addProperty("propMaxTorque", propMaxTorque);
    addProperty("propId", propId);
    addProperty("propDynamixelFamily", propDynamixelFamily);

    addPort("outState", outState);
}
}
