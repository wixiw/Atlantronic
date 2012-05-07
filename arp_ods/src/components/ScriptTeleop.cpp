/*
 * ScriptTeleop.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "ScriptTeleop.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::ScriptTeleop )

ScriptTeleop::ScriptTeleop(const std::string& name):
        OdsTaskContext(name),
        propLinSpeed(0.5),
        propAngSpeed(0.020),
        attrMode(0),
        attrTwistCmd(0,0,0),
        m_direction(0.0)
{
    addProperty("propLinSpeed",propLinSpeed);
    addProperty("propAngSpeed",propAngSpeed);
    addAttribute("attrMode",attrMode);
    addAttribute("attrTwistCmd",attrTwistCmd);
    addPort("outTwistCmd",outTwistCmd)
            .doc("");
}

void ScriptTeleop::updateHook()
{
    OdsTaskContext::updateHook();
    Twist2D twist;

    switch (attrMode) {
        case 0:
            twist = attrTwistCmd;
            break;
        case 1:
            m_direction = m_direction + propAngSpeed;
            twist = MathFactory::createTwist2DFromPolarRepr(propLinSpeed,m_direction, 0);
            break;
        default:
            break;
    }

    outTwistCmd.write(twist);
}

