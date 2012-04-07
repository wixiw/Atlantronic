/*
 * TwistTeleop.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "TwistTeleop.hpp"
#include <rtt/Component.hpp>

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

    addPort("outTwistCmd",outTwistCmd)
            .doc("");
}

void TwistTeleop::updateHook()
{
    OdsTaskContext::updateHook();

    double vx,vy,vtheta;

    inXSpeed.readNewest(vx);
    inYSpeed.readNewest(vy);
    inThetaSpeed.readNewest(vtheta);

    Twist2D twist(-vy*propLinearGain,-vx*propLinearGain,-vtheta*propAngularGain);

    outTwistCmd.write(twist);
}

