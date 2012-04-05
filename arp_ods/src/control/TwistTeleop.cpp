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
        OdsTaskContext(name)
{

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

    inXSpeed.readNewest(vy);
    inYSpeed.readNewest(vx);
    inThetaSpeed.readNewest(vtheta);

    Twist2D twist(-vx,vy,vtheta);

    outTwistCmd.write(twist);
}

