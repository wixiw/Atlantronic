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

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace RTT;
using namespace std;


ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name),
        propMinSpeed(0.001)
{
    createOrocosInterface();
    arp_model::Logger::InitFile("arp_model", DEBUG);
    //arp_model::Logger::InitConsole("arp_model", DEBUG);
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
    double dt = 0.030; //TODO faire mieux ! mettre un vrai temps calculé depuis la derniere execution


//TODO remettre le filtre quand il marche    //filter the input command to get a reachable command that we are sure the hardware will be capable to do
//    if( KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
//                                    attrMotorsCurrentState, attrParams,
//                                    dt, propMinSpeed, attrAcceptableTwist, attrQuality) == false )
//    {
//        LOG(Error) << "Failed to filter desired twist to an acceptable twist" << endlog();
//    }

    //TODO à supprimer quand le filtre marche : ceci annule le filtrage
    attrAcceptableTwist = attrTwistCmd;

    //compute the motor commands to do the filtered twist command
    if( UbiquityKinematics::twist2Motors(attrAcceptableTwist, attrMotorsCurrentState, attrTurretState, attrMotorStateCommand, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    Log(INFO) << "welcome back to kinematic base----- ";
    Log(INFO) << "acceptable Twist=                                "<< attrAcceptableTwist.toString() << " phi=" << attrAcceptableTwist.speedAngle() << " v=" << attrAcceptableTwist.speedNorm();
    Log(INFO) << "command for steering motors                 :    " << "("<< attrMotorStateCommand.steering.left.position << " , " << attrMotorStateCommand.steering.right.position << " , " << attrMotorStateCommand.steering.rear.position << ")";
    Log(INFO) << "command for turrets                 :            " <<"("<< attrTurretState.steering.left.position << " , " << attrTurretState.steering.right.position << " , " << attrTurretState.steering.rear.position << ")";

    //gestion du cas pas de vitesse pour ne pas bouger les tourelles
    //TODO a supprimer avec le filtre ?
    if( attrAcceptableTwist.speedNorm() <= propMinSpeed )
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

    addProperty("propMinSpeed",propMinSpeed)
            .doc("");

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
