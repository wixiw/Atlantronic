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

    addPort("inLeftSteeringVelocityMeasure", inLeftSteeringVelocityMeasure)
            .doc("");
    addPort("inRightSteeringVelocityMeasure", inRightSteeringVelocityMeasure)
            .doc("");
    addPort("inRearSteeringVelocityMeasure", inRearSteeringVelocityMeasure)
            .doc("");

    addPort("inLeftSteeringPositionMeasure", inLeftSteeringPositionMeasure)
            .doc("");
    addPort("inRightSteeringPositionMeasure", inRightSteeringPositionMeasure)
            .doc("");
    addPort("inRearSteeringPositionMeasure", inRearSteeringPositionMeasure)
            .doc("");

    addPort("inLeftDrivingVelocityMeasure", inLeftDrivingVelocityMeasure)
            .doc("");
    addPort("inRightDrivingVelocityMeasure", inRightDrivingVelocityMeasure)
            .doc("");
    addPort("inRearDrivingVelocityMeasure", inRearDrivingVelocityMeasure)
            .doc("");


    addPort("outLeftDrivingVelocityCmd",outLeftDrivingVelocityCmd)
            .doc("");
    addPort("outRightDrivingVelocityCmd",outRightDrivingVelocityCmd)
            .doc("");
    addPort("outRearDrivingVelocityCmd",outRearDrivingVelocityCmd)
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
    SteeringMotorVelocities turretVelocities;
    UbiquityParams params;


    // gathering of Steering Velocities
    inLeftSteeringVelocityMeasure.readNewest(turretVelocities.leftSteeringMotorVelocity);
    inRightSteeringVelocityMeasure.readNewest(turretVelocities.rightSteeringMotorVelocity);
    inRearSteeringVelocityMeasure.readNewest(turretVelocities.rearSteeringMotorVelocity);

    // gathering of motors positions and speeds
    inLeftSteeringPositionMeasure.readNewest(attrMotorsCurrentState.leftSteeringMotorPosition);
    inRightSteeringPositionMeasure.readNewest(attrMotorsCurrentState.rightSteeringMotorPosition);
    inRearSteeringPositionMeasure.readNewest(attrMotorsCurrentState.rearSteeringMotorPosition);
    inLeftDrivingVelocityMeasure.readNewest(attrMotorsCurrentState.leftDrivingMotorVelocity);
    inRightDrivingVelocityMeasure.readNewest(attrMotorsCurrentState.rightDrivingMotorVelocity);
    inRearDrivingVelocityMeasure.readNewest(attrMotorsCurrentState.rearDrivingMotorVelocity);

    inTwistCmd.readNewest(attrTwistCmd);
    inCurrentTwist.readNewest(attrCurrentTwist);
    inParams.readNewest(params);

    // COMPUTATIONS
    TurretState turretCmd;
    MotorState motorCmd;

    // BMO pour RMO : Il faut récupérer les infos Moteur, les convertir en infos Tourelle (gestion du couplage des axes en particulier).
    // Il y a une fonction pour ça dans UbiquityKinematics. Par contre, il va te manquer des infos (l'état courant de la tourelle).
    // Il va donc te falloir de nouveaux ports.
//    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist, attrMotorsCurrentState, attrAcceptableTwist, params) == false )
//        {
//            LOG(Error) << "Failed to filter desired twist to an acceptable twist" << endlog();
//        }

    if( UbiquityKinematics::twist2Turrets(attrAcceptableTwist, turretCmd, params) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }
    else if( UbiquityKinematics::turrets2Motors(turretCmd, turretVelocities, motorCmd, params) == false )
    {
        LOG(Error) << "Failed to compute Motor Cmd" << endlog();
    }

    LOG(Info) << "turrets : " << turretCmd.toString() << endlog();

    outLeftDrivingVelocityCmd.write(motorCmd.leftDrivingMotorVelocity);
    outRightDrivingVelocityCmd.write(motorCmd.rightDrivingMotorVelocity);
    outRearDrivingVelocityCmd.write(motorCmd.rearDrivingMotorVelocity);
    outLeftSteeringPositionCmd.write(motorCmd.leftSteeringMotorPosition);
    outRightSteeringPositionCmd.write(motorCmd.rightSteeringMotorPosition);
    outRearSteeringPositionCmd.write(motorCmd.rearSteeringMotorPosition);
}
