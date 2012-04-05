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
    addAttribute("attrMotorsCurrentState",attrMotorsCurrentState);

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

    addPort("inLeftSteeringPositionMeasure", inLeftSteeringPositionMeasure)
            .doc("");
    addPort("inRightSteeringPositionMeasure", inRightSteeringPositionMeasure)
            .doc("");
    addPort("inRearSteeringPositionMeasure", inRearSteeringPositionMeasure)
            .doc("");

    addPort("inLeftDrivingSpeedMeasure", inLeftDrivingSpeedMeasure)
            .doc("");
    addPort("inRightDrivingSpeedMeasure", inRightDrivingSpeedMeasure)
            .doc("");
    addPort("inRearDrivingSpeedMeasure", inRearDrivingSpeedMeasure)
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

    // INPUTS
    CouplingSpeeds turretSpeeds;
    UbiquityParams params;


    // gathering of Steering Speeds
    inLeftSteeringSpeedMeasure.readNewest(turretSpeeds.leftSteeringMotorSpeed);
    inRightSteeringSpeedMeasure.readNewest(turretSpeeds.rightSteeringMotorSpeed);
    inRearSteeringSpeedMeasure.readNewest(turretSpeeds.rearSteeringMotorSpeed);

    // gathering of motors positions and speeds
    inLeftSteeringPositionMeasure.readNewest(attrMotorsCurrentState.leftSteeringMotorPosition);
    inRightSteeringPositionMeasure.readNewest(attrMotorsCurrentState.rightSteeringMotorPosition);
    inRearSteeringPositionMeasure.readNewest(attrMotorsCurrentState.rearSteeringMotorPosition);
    inLeftDrivingSpeedMeasure.readNewest(attrMotorsCurrentState.leftDrivingMotorSpeed);
    inRightDrivingSpeedMeasure.readNewest(attrMotorsCurrentState.rightDrivingMotorSpeed);
    inRearDrivingSpeedMeasure.readNewest(attrMotorsCurrentState.rearDrivingMotorSpeed);

    inTwistCmd.readNewest(attrTwistCmd);
    inCurrentTwist.readNewest(attrCurrentTwist);
    inParams.readNewest(params);

    // COMPUTATIONS
    TurretCommands turretCmd;
    MotorCommands motorCmd;

    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist, attrMotorsCurrentState, attrAcceptableTwist, params) == false )
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
