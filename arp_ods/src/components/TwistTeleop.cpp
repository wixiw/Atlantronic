/*
 * TwistTeleop.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "TwistTeleop.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace arp_math;
using namespace arp_model;
using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::TwistTeleop )

TwistTeleop::TwistTeleop(const std::string& name):
        OdsTaskContext(name),
        propLinearGain(0.3),
        propAngularGain(3),
        propLinearAcc(1),
        propAngularAcc(1),
        propFilter(true)
{
    addAttribute("attrInTwist",attrLastTwist);
    addAttribute("attrTwistCmdCdg",attrTwistCmdCdg);
    addAttribute("attrTwistCmdB4Filter",attrTwistCmdB4Filter);
    addProperty("propLinearGain",propLinearGain);
    addProperty("propAngularGain",propAngularGain);
    addProperty("propLinearAcc",propLinearAcc);
    addProperty("propAngularAcc",propAngularAcc);
    addProperty("propFilter", propFilter)
        .doc("Set this to true if you want acc/dec filters to operates (so it becomes a closed loop). else it only send command in open loop");

    addPort("inXSpeed",inXSpeed)
            .doc("");
    addPort("inYSpeed",inYSpeed)
            .doc("");
    addPort("inThetaSpeed",inThetaSpeed)
            .doc("");
    addPort("inParams",inParams)
            .doc("");
    addPort("inTwist",inTwist)
            .doc("");
    addPort("outTwistCmd",outTwistCmd)
            .doc("");
}

void TwistTeleop::updateHook()
{
    OdsTaskContext::updateHook();
    UbiquityParams params;
    double vx,vy,vtheta;

    inXSpeed.readNewest(vx);
    inYSpeed.readNewest(vy);
    inThetaSpeed.readNewest(vtheta);
    inParams.readNewest(params);
    inTwist.readNewest(attrLastTwist);


    //on veut piloter en "carthesien" au centre du robot
    Twist2D twistCdg(-vy*propLinearGain,-vx*propLinearGain,-vtheta*propAngularGain);
    attrTwistCmdCdg = twistCdg;
    //mais le robot se pilote au centre des tourelles :(
    Twist2D attrTwistCmdB4Filter = twistCdg.transport(params.getChassisCenter().inverse());
    Twist2D twistRef = attrTwistCmdB4Filter;

        outTwistCmd.write(twistRef);


}

