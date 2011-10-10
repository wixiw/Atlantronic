/*
 * UbiquityItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "UbiquitySimul.hpp"
#include <rtt/Component.hpp>

using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::UbiquitySimul )

UbiquitySimul::UbiquitySimul(const std::string& name):
UbiquityItf(name)
{
}

bool UbiquitySimul::configureHook()
{
    bool res = true;
    HmlTaskContext::configureHook();
}

void UbiquitySimul::updateHook()
{
    UbiquityItf::writeOmniCmd();

    loopEncoder();
}

void UbiquitySimul::loopEncoder()
{
    attrOdometers.odo_left_driving = 0;
    attrOdometers.odo_right_driving = 0;
    attrOdometers.odo_rear_driving = 0;
    attrOdometers.odo_left_steering = 0;
    attrOdometers.odo_right_steering = 0;
    attrOdometers.odo_rear_steering = 0;
    attrOdometers.time = 0;
    outOdometryMeasures.write(attrOdometers);
}
