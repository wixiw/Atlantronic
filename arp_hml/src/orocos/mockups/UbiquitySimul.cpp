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
    addAttribute("attrRealPosition",attrRealPosition);

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

    addOperation("ooSetRealSimulPosition", &UbiquitySimul::setPosition,this, OwnThread )
            .doc("Define a new absolute real robot position");

}

bool UbiquitySimul::configureHook()
{
    bool res = HmlTaskContext::configureHook();
    return res;
}

void UbiquitySimul::updateHook()
{
    outRealPosition.write(attrRealPosition);
}

bool UbiquitySimul::setPosition(Pose2D newPose)
{
    LOG(Info) << "Setting new Real Position to " << newPose.toString() << endlog();
    attrRealPosition.x(newPose.x());
    attrRealPosition.y(newPose.y());
    attrRealPosition.h(newPose.h());
    return true;
}
