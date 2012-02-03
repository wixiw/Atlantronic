/*
 * HmlCmdMockup.cpp
 *
 *  Created on: 03 fev. 2012
 *      Author: wla
 */

#include "HmlCmdMockup.hpp"
#include <rtt/Component.hpp>

using namespace arp_hml;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::HmlCmdMockup )

HmlCmdMockup::HmlCmdMockup(const std::string& name):
    HmlTaskContext(name)
{
    addPort("outLeftDrivingSpeedCmd",outLeftDrivingSpeedCmd);
    addPort("outRightDrivingSpeedCmd",outRightDrivingSpeedCmd);
    addPort("outRearDrivingSpeedCmd",outRearDrivingSpeedCmd);
    addPort("outLeftSteeringSpeedCmd",outLeftSteeringSpeedCmd);
    addPort("outRightSteeringSpeedCmd",outRightSteeringSpeedCmd);
    addPort("outRearSteeringSpeedCmd",outRearSteeringSpeedCmd);

    addPort("outLeftDrivingPositionCmd",outLeftDrivingPositionCmd);
    addPort("outRightDrivingPositionCmd",outRightDrivingPositionCmd);
    addPort("outRearDrivingPositionCmd",outRearDrivingPositionCmd);
    addPort("outLeftSteeringPositionCmd",outLeftSteeringPositionCmd);
    addPort("outRightSteeringPositionCmd",outRightSteeringPositionCmd);
    addPort("outRearSteeringPositionCmd",outRearSteeringPositionCmd);

    addPort("outLeftDrivingTorqueCmd",outLeftDrivingTorqueCmd);
    addPort("outRightDrivingTorqueCmd",outRightDrivingTorqueCmd);
    addPort("outRearDrivingTorqueCmd",outRearDrivingTorqueCmd);
    addPort("outLeftSteeringTorqueCmd",outLeftSteeringTorqueCmd);
    addPort("outRightSteeringTorqueCmd",outRightSteeringTorqueCmd);
    addPort("outRearSteeringTorqueCmd",outRearSteeringTorqueCmd);

    addPort("outBit01",outBit01);
    addPort("outBit02",outBit02);
    addPort("outBit03",outBit03);
    addPort("outBit04",outBit04);
    addPort("outBit05",outBit05);
    addPort("outBit06",outBit06);
    addPort("outBit07",outBit07);
    addPort("outBit08",outBit08);
}

