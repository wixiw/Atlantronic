/*
 * UbiquitySimul.cpp
 *
 *  Created on: 02 April 2012
 *      Author: wla
 */

#include "UbiquitySimul.hpp"
#include <rtt/Component.hpp>

using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::UbiquitySimul )

UbiquitySimul::UbiquitySimul(const std::string& name):
    HmlTaskContext(name)
{
    addPort("inLeftSteeringPositionCmd",inLeftSteeringPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");
    addPort("inRightSteeringPositionCmd",inRightSteeringPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");
    addPort("inRearSteeringPositionCmd",inRearSteeringPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");

    addPort("inLeftDrivingSpeedCmd",inLeftDrivingSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");
    addPort("inRightDrivingSpeedCmd",inRightDrivingSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");
    addPort("inRearDrivingSpeedCmd",inRearDrivingSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");

    addPort("outRealPosition",outRealPosition)
        .doc("Provides the real position of the robot taking into account the commands ");
}

bool UbiquitySimul::configureHook()
{
    bool res = HmlTaskContext::configureHook();
    return res;
}

void UbiquitySimul::updateHook()
{
    static Pose2D p(1,2,3);
    p.x(p.x()+1);
    outRealPosition.write(p);
}

