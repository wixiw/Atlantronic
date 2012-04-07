/*
 * TwistTeleop.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "TwistTeleop.hpp"
#include <rtt/Component.hpp>

using namespace RTT;
using namespace arp_math;
using namespace arp_model;
using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::TwistTeleop )

TwistTeleop::TwistTeleop(const std::string& name):
        OdsTaskContext(name),
        propLinearGain(0.3),
        propAngularGain(1)
{
    addProperty("propLinearGain",propLinearGain);
    addProperty("propAngularGain",propAngularGain);

    addPort("inXSpeed",inXSpeed)
            .doc("");
    addPort("inYSpeed",inYSpeed)
            .doc("");
    addPort("inThetaSpeed",inThetaSpeed)
            .doc("");
    addPort("inParams",inParams)
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

    //on veut piloter en "carthesien" au centre du robot
    Twist2D twistCdg(-vy*propLinearGain,-vx*propLinearGain,-vtheta*propAngularGain);

    //mais le robot se pilote au centre des tourelles :(
    Twist2D twistRef = twistCdg.transport(params.getChassisCenter().inverse());

    outTwistCmd.write(twistRef);
}

