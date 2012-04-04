/*
 * KinematicBase.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "KinematicBase.hpp"
#include <rtt/Component.hpp>
#include <models/UbiquityKinematics.hpp>

using namespace arp_core;
using namespace arp_math;
using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name),
        inClock()
{
    addEventPort("inClock",inClock)
            .doc("Clock port which trigger our activity. It contains the time at which the input data are supposed to be calculated");

    addPort("inTwistCmd",inTwistCmd)
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

    Twist2D twistCmd;
    TurretCommands turretCmd;
    MotorCommands motorCmd;

    CouplingSpeeds turretSpeeds;
    inLeftSteeringSpeedMeasure.readNewest(turretSpeeds.leftSteeringMotorSpeed);
    inRightSteeringSpeedMeasure.readNewest(turretSpeeds.rightSteeringMotorSpeed);
    inRearSteeringSpeedMeasure.readNewest(turretSpeeds.rearSteeringMotorSpeed);

    UbiquityParams params;

    inTwistCmd.readNewest(twistCmd);
    inParams.readNewest(params);

    if( UbiquityKinematics::twist2Turrets(twistCmd, turretCmd, params) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }
    else if( UbiquityKinematics::turrets2Motors(turretCmd, motorCmd, turretSpeeds, params) == false )
    {
        LOG(Error) << "Failed to compute Motor Cmd" << endlog();
    }

    outLeftDrivingSpeedCmd.write(motorCmd.leftDrivingMotorSpeed);
    outRightDrivingSpeedCmd.write(motorCmd.rightDrivingMotorSpeed);
    outRearDrivingSpeedCmd.write(motorCmd.rearDrivingMotorSpeed);
    outLeftSteeringPositionCmd.write(motorCmd.leftSteeringMotorPosition);
    outRightSteeringPositionCmd.write(motorCmd.rightSteeringMotorPosition);
    outRearDrivingSpeedCmd.write(motorCmd.rearSteeringMotorPosition);
}
