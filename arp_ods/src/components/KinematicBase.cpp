/*
 * KinematicBase.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "KinematicBase.hpp"
#include <rtt/Component.hpp>
#include <models/core>
#include "control/KinematicFilter.hpp"

using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name)
{
    addAttribute("attrTwistCmd",attrTwistCmd);
    addAttribute("attrCurrentTwist",attrCurrentTwist);
    addAttribute("attrAcceptableTwist",attrAcceptableTwist);
    addAttribute("attrMotorsCurrentState",attrMotorsCurrentState);

    addEventPort("inTwistCmd",inTwistCmd)
            .doc("");
    addPort("inCurrentTwist",inCurrentTwist)
            .doc("");
    addPort("inParams",inParams)
            .doc("");

    addPort("inMotorState", inMotorState)
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
    inMotorState.readNewest(attrMotorsCurrentState);
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
    //WLA pour BMO : tu as oublié de faire le fitlre Identité ^ ^ à la place du filtre à Moumou
    attrAcceptableTwist = attrTwistCmd;

    if( UbiquityKinematics::twist2Motors(attrAcceptableTwist, turretVelocities, motorCmd, params) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    //LOG(Info) << "turrets : " << turretCmd.toString() << endlog();

    outLeftDrivingVelocityCmd.write(motorCmd.leftDrivingVelocity);
    outRightDrivingVelocityCmd.write(motorCmd.rightDrivingVelocity);
    outRearDrivingVelocityCmd.write(motorCmd.rearDrivingVelocity);
    outLeftSteeringPositionCmd.write(motorCmd.leftSteeringPosition);
    outRightSteeringPositionCmd.write(motorCmd.rightSteeringPosition);
    outRearSteeringPositionCmd.write(motorCmd.rearSteeringPosition);
}
