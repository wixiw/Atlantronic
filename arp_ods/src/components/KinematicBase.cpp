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
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name)
{
    createOrocosInterface();
}

void KinematicBase::updateHook()
{
    OdsTaskContext::updateHook();
    getInputs();
    run();
    setOutputs();
}

void KinematicBase::getInputs()
{
    //buffering ports
    EstimatedTwist2D inTwist;
    inMotorState.readNewest(attrMotorsCurrentState);
    inTwistCmd.readNewest(attrTwistCmd);
    inCurrentTwist.readNewest(inTwist);
    attrCurrentTwist = inTwist.toTwist();
    inParams.readNewest(attrParams);

    //initialisation of attribute to something realistic just in case
    attrAcceptableTwist = attrCurrentTwist;
}

void KinematicBase::run()
{
    double dt = 0.010; //TODO faire mieux ! mettre un vrai temps calculé depuis la derniere execution

    /*
    //filter the input command to get a reachable command that we are sure the hardware will be capable to do
    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, attrAcceptableTwist, attrQuality) == false )
    {
        //TODO remettre en erreur
        LOG(Info) << "Failed to filter desired twist to an acceptable twist" << endlog();
    }*/
    attrAcceptableTwist = attrTwistCmd;

    //compute the motor commands to do the filtered twist command
    if( UbiquityKinematics::twist2Motors(attrAcceptableTwist, attrMotorsCurrentState, attrTurretState, attrMotorStateCommand, attrParams) == false )
    {
        //TODO remettre en erreur
        LOG(Info) << "Failed to compute Turrets Cmd" << endlog();
    }

    //gestion du cas pas de vitesse pour ne pas bouger les tourelles
    //TODO mettre en propriete
    if( attrAcceptableTwist.speedNorm() <= 0.001 )
    {
        attrMotorStateCommand.steering = attrMotorsCurrentState.steering;
    }
}

void KinematicBase::setOutputs()
{
    outLeftDrivingVelocityCmd.write(attrMotorStateCommand.driving.left.velocity);
    outRightDrivingVelocityCmd.write(attrMotorStateCommand.driving.right.velocity);
    outRearDrivingVelocityCmd.write(attrMotorStateCommand.driving.rear.velocity);
    outLeftSteeringPositionCmd.write(attrMotorStateCommand.steering.left.position);
    outRightSteeringPositionCmd.write(attrMotorStateCommand.steering.right.position);
    outRearSteeringPositionCmd.write(attrMotorStateCommand.steering.rear.position);
    outFiltrationFeedback.write(attrQuality);
    outFilteredTwist.write(attrAcceptableTwist);
}

void KinematicBase::createOrocosInterface()
{
    addAttribute("attrTwistCmd",attrTwistCmd);
    addAttribute("attrCurrentTwist",attrCurrentTwist);
    addAttribute("attrAcceptableTwist",attrAcceptableTwist);
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorStateCommand",attrMotorStateCommand);
    addAttribute("attrMotorsCurrentState",attrMotorsCurrentState);
    addAttribute("attrParams",attrParams);
    addAttribute("attrQuality",attrQuality);

    addEventPort("inTwistCmd",inTwistCmd)
            .doc("");
    addPort("inCurrentTwist",inCurrentTwist)
            .doc("");
    addPort("inMotorState", inMotorState)
             .doc("");
    addPort("inParams",inParams)
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
