/*
 * UbiquityItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "UbiquitySimul.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::UbiquitySimul )

UbiquitySimul::UbiquitySimul(const std::string& name):
UbiquityItf(name)
{
}

bool UbiquitySimul::configureHook()
{
    bool res = true;
    return HmlTaskContext::configureHook();
}

void UbiquitySimul::updateHook()
{
    loopEncoder();
}

void UbiquitySimul::loopEncoder()
{
    OmniCommand cmd;
    if(NewData==inOmniCmd.readNewest(cmd))
    {
       attrCurrentCmd = cmd;
    }

    attrOdometers.odo_left_driving = attrCurrentCmd.v_left_driving;
    attrOdometers.odo_right_driving = attrCurrentCmd.v_right_driving;
    attrOdometers.odo_rear_driving = attrCurrentCmd.v_rear_driving;
    attrOdometers.odo_left_steering = attrCurrentCmd.v_left_steering;
    attrOdometers.odo_right_steering = attrCurrentCmd.v_right_steering;
    attrOdometers.odo_rear_steering = attrCurrentCmd.v_rear_steering;
    attrOdometers.time = arp_math::getTime();
    outOdometryMeasures.write(attrOdometers);
}
