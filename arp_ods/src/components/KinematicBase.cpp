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
    inMotorState.readNewest(attrMotorsCurrentState);
    inTwistCmd.readNewest(attrTwistCmd);
    inCurrentTwist.readNewest(attrCurrentTwist);
    inParams.readNewest(attrParams);

    //initialisation of attribute to something realistic just in case
    attrAcceptableTwist = attrCurrentTwist;
}

void KinematicBase::run()
{
    // INPUTS
    SteeringMotorVelocities turretVelocities;
    double dt = 0.010; //TODO faire mieux ! mettre un vrai temps calculé depuis la derniere execution

    //filter the input command to get a reachable command that we are sure the hardware will be capable to do
    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, attrAcceptableTwist, attrQuality) == false )
    {
        LOG(Error) << "Failed to filter desired twist to an acceptable twist" << endlog();
    }

    //compute the motor commands to do the filtered twist command
    if( UbiquityKinematics::twist2Motors(attrAcceptableTwist, turretVelocities, attrMotorStateCommand, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }
}

void KinematicBase::setOutputs()
{
    outLeftDrivingVelocityCmd.write(attrMotorStateCommand.leftDrivingVelocity);
    outRightDrivingVelocityCmd.write(attrMotorStateCommand.rightDrivingVelocity);
    outRearDrivingVelocityCmd.write(attrMotorStateCommand.rearDrivingVelocity);
    outLeftSteeringPositionCmd.write(attrMotorStateCommand.leftSteeringPosition);
    outRightSteeringPositionCmd.write(attrMotorStateCommand.rightSteeringPosition);
    outRearSteeringPositionCmd.write(attrMotorStateCommand.rearSteeringPosition);
    outFiltrationFeedback.write(attrQuality);
    outFilteredTwist.write(attrAcceptableTwist);
}

void KinematicBase::createOrocosInterface()
{
    addAttribute("attrTwistCmd",attrTwistCmd);
    addAttribute("attrCurrentTwist",attrCurrentTwist);
    addAttribute("attrAcceptableTwist",attrAcceptableTwist);
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
