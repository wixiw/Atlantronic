/*
 * KinematicBase.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "KinematicBase.hpp"
#include <rtt/Component.hpp>
#include <models/UbiquityKinematics.hpp>
#include "KinematicFilter.hpp"

using namespace arp_core;
using namespace arp_math;
using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name),
        inClock()
{
    addAttribute("attrTwistCmd",attrTwistCmd);
    addAttribute("attrCurrentTwist",attrCurrentTwist);
    addAttribute("attrAcceptableTwist",attrAcceptableTwist);

    addEventPort("inClock",inClock)
            .doc("Clock port which trigger our activity. It contains the time at which the input data are supposed to be calculated");
    addPort("inTwistCmd",inTwistCmd)
            .doc("");
    addPort("inCurrentTwist",inCurrentTwist)
            .doc("");
    addPort("inParams",inParams)
            .doc("");

    addPort("inLeftSteeringSpeedMeasure", inLeftSteeringSpeedMeasure)
            .doc("");
    addPort("inRightSteeringSpeedMeasure", inRightSteeringSpeedMeasure)
            .doc("");
    addPort("inRearSteeringSpeedMeasure", inRearSteeringSpeedMeasure)
            .doc("");

    addPort("outLeftDrivingSpeedCmd",outLeftDrivingSpeedCmd)
            .doc("");
    addPort("outRightDrivingSpeedCmd",outRightDrivingSpeedCmd)
            .doc("");
    addPort("outRearDrivingSpeedCmd",outRearDrivingSpeedCmd)
            .doc("");
    addPort("outLeftSteeringPositionCmd",outLeftSteeringPositionCmd)
            .doc("");
    addPort("outRightSteeringPositionCmd",outRightSteeringPositionCmd)
            .doc("");
    addPort("outRearSteeringPositionCmd",outRearSteeringPositionCmd)
            .doc("");

}

void KinematicBase::updateHook()
{
    OdsTaskContext::updateHook();

    TurretCommands turretCmd;
    MotorCommands motorCmd;
    CouplingSpeeds turretSpeeds;
    UbiquityParams params;

    inLeftSteeringSpeedMeasure.readNewest(turretSpeeds.leftSteeringMotorSpeed);
    inRightSteeringSpeedMeasure.readNewest(turretSpeeds.rightSteeringMotorSpeed);
    inRearSteeringSpeedMeasure.readNewest(turretSpeeds.rearSteeringMotorSpeed);
    inTwistCmd.readNewest(attrTwistCmd);
    inCurrentTwist.readNewest(attrCurrentTwist);
    inParams.readNewest(params);

    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist, attrAcceptableTwist, params) == false )
        {
            LOG(Error) << "Failed to filter desired twist to an acceptable twist" << endlog();
        }

    if( UbiquityKinematics::twist2Turrets(attrAcceptableTwist, turretCmd, params) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }
    else if( UbiquityKinematics::turrets2Motors(turretCmd, motorCmd, turretSpeeds, params) == false )
    {
        LOG(Error) << "Failed to compute Motor Cmd" << endlog();
    }

    LOG(Info) << "turrets : " << turretCmd << endlog();

    outLeftDrivingSpeedCmd.write(motorCmd.leftDrivingMotorSpeed);
    outRightDrivingSpeedCmd.write(motorCmd.rightDrivingMotorSpeed);
    outRearDrivingSpeedCmd.write(motorCmd.rearDrivingMotorSpeed);
    outLeftSteeringPositionCmd.write(motorCmd.leftSteeringMotorPosition);
    outRightSteeringPositionCmd.write(motorCmd.rightSteeringMotorPosition);
    outRearSteeringPositionCmd.write(motorCmd.rearSteeringMotorPosition);
}
