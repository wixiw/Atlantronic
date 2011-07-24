/*
 * Logitech3DTeleop.cpp
 *
 *  Created on: 01 may 2011
 *      Author: wla
 */

#include "Logitech3DTeleop.hpp"
#include <ocl/Component.hpp>
#include <ros/package.h>

using namespace arp_ods;
using namespace arp_core;
using namespace std_msgs;

ORO_LIST_COMPONENT_TYPE( arp_ods::Logitech3DTeleop)

Logitech3DTeleop::Logitech3DTeleop(const std::string& name) :
    ARDTaskContext(name, ros::package::getPath("arp_ods")), propLongGain(1),
            propRotGain(1)
{
    addAttribute("attrVelocityCommand", attrVelocityCommand);
    addPort("inY", inY).doc(
            "Value between [-1;1] proportionnal to joystick far-close Y-axis");
    addPort("inZ", inZ).doc(
            "Value between [-1;1] proportionnal to joystick rotation clock-wise Z-axis");
    addPort("inDeadMan", inDeadMan);
    addPort("inPower", inPower);
    addPort("outVelocityCmd", outVelocityCmd).doc("");

    addProperty("propLongGain", propLongGain);
    addProperty("propRotGain", propRotGain);
}

Logitech3DTeleop::~Logitech3DTeleop()
{
}

bool  Logitech3DTeleop::configureHook()
{
    bool res = ARDTaskContext::configureHook();

     res &= getOperation("Protokrot", "ooSetMotorPower", m_ooSetPower);

    return res;
}

void Logitech3DTeleop::updateHook()
{
    ARDTaskContext::updateHook();
    Velocity velocityCommand;
    double longSpeed;
    double rotSpeed;
    inY.readNewest(longSpeed);
    inZ.readNewest(rotSpeed);
    Bool power;

    bool deadMan;
    inDeadMan.readNewest(deadMan);
    inPower.readNewest(power);

    if (deadMan == true)
    {
        attrVelocityCommand.linear = -propLongGain * longSpeed;
        attrVelocityCommand.angular = -propRotGain * rotSpeed;

        if( power.data == false )
        {
            if( !m_ooSetPower(true,1) )
                LOG(Error) << "Failed to set motor power on" << endlog();
        }
    }
    else
    {
        attrVelocityCommand.linear = 0;
        attrVelocityCommand.angular = 0;

        if( power.data == true )
        {
            if( !m_ooSetPower(false,1) )
                LOG(Error) << "Failed to set motor power off" << endlog();
        }
    }
    outVelocityCmd.write(attrVelocityCommand);
}
