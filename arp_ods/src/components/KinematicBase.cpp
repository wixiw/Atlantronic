/*
 * KinematicBase.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "KinematicBase.hpp"
#include <rtt/Component.hpp>
#include <models/core>
#include "../control/planners/KinematicFilter.hpp"

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase)

KinematicBase::KinematicBase(const std::string& name) :
        OdsTaskContext(name), attrRobotBlockedTimeout(false), attrBlockTime(0), propRobotBlockedTimeout(1.0), propMaxSpeedDiff(0.200)
{
    createOrocosInterface();
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    //arp_model::Logger::InitFile("arp_model", INFO);
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
    EstimatedICRSpeed inICRSpeed;
    inMotorState.readNewest(attrMotorsCurrentState);
    inICRSpeedCmd.readNewest(attrICRSpeedCmd);
    inCurrentICRSpeed.readNewest(inICRSpeed);
    attrCurrentICRSpeed = (ICRSpeed) inICRSpeed;
    inParams.readNewest(attrParams);

}

void KinematicBase::run()
{
    Log(DEBUG) << ">> KinematicBase::run()   -----------------------------";

    //check robot blocked with new speeds and last commands
    checkRobotBlocked();

    //compute the motor commands from ICRSpeed input
    if (UbiquityKinematics::ICRSpeed2Motors(attrICRSpeedCmd, attrMotorsCurrentState, attrTurretState,
            attrMotorStateCommand, attrParams) == false)
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    //check if the command is reachable
    //TODO TEMPS EN DUR !!!
    double dt = 0.010;
    if(false == KinematicFilter::isMotorStateReachable(attrMotorStateCommand,attrMotorsCurrentState,attrParams, dt))
    {
    //    LOG(Error) << "Commanded ICRSpeed is not reachable" << endlog();
    }
}

void KinematicBase::checkRobotBlocked()
{
    //comparison of measured twist to the commanded twist. if they are not consistent then the robot is not able to perform its motion - there is a problem..

    //if robot is blocked and timer is 0 then we just began to block
    if (!consistencyMeasuredvsCommanded())
    {
        if (attrBlockTime == 0)
        {
            attrBlockTime = getTime();
        }
        else
        {
            double delay = getTime() - attrBlockTime;
            if (delay < 0 || delay > propRobotBlockedTimeout)
            {
                Log(INFO) << "Kinematic base detected robot blocage";
                attrRobotBlockedTimeout = true;
            }
        }
    }
    else
    {
        attrBlockTime = 0;
        attrRobotBlockedTimeout = false;
    }
}

bool KinematicBase::consistencyMeasuredvsCommanded()
{
    //TODO a refaire
    // distance bewteen twist. thetap is given a coefficient 0.04=0.2² to represent the speed at a 20cm lever


    //double speederror = attrCurrentICRSpeed.distanceTo(attrAcceptableICRSpeed, 1.0, 0.2);

    //////////////////////
//    Log(DEBUG) << ">> KinematicBase::consistencyMeasuredvsCommanded()";
//    Log(DEBUG) << "command  : "<<attrAcceptableTwist.toString();
//    Log(DEBUG) << "measure  : "<<attrCurrentTwist.toString();
//    Log(DEBUG) << "error    : "<<speederror;
//    Log(DEBUG) << "<< KinematicBase::consistencyMeasuredvsCommanded()";
    ////////////////////

    // 20 mm/s difference accepted
    //return speederror < propMaxSpeedDiff;

    return true;
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
    outFilteredICRSpeed.write(attrCurrentICRSpeed);
    outRobotBlocked.write(attrRobotBlockedTimeout);
}

void KinematicBase::createOrocosInterface()
{
    addAttribute("attrICRSpeedCmd", attrICRSpeedCmd);
    addAttribute("attrCurrentICRSpeed", attrCurrentICRSpeed);
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorStateCommand", attrMotorStateCommand);
    addAttribute("attrMotorsCurrentState", attrMotorsCurrentState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrQuality", attrQuality);
    addAttribute("attrRobotBlockedTimeout", attrRobotBlockedTimeout);
    addAttribute("attrBlockTime", attrBlockTime);

    addProperty("propRobotBlockedTimeout", propRobotBlockedTimeout).doc("");
    addProperty("propMaxSpeedDiff", propMaxSpeedDiff).doc(
            "Max distance between commanded and measured Speed before declaring the robot is blocked (in m/s)");

    addPort("inICRSpeedCmd", inICRSpeedCmd).doc("");
    addPort("inCurrentICRSpeed", inCurrentICRSpeed).doc("");
    addPort("inMotorState", inMotorState).doc("");
    addPort("inParams", inParams).doc("");

    addPort("outLeftDrivingVelocityCmd", outLeftDrivingVelocityCmd).doc("");
    addPort("outRightDrivingVelocityCmd", outRightDrivingVelocityCmd).doc("");
    addPort("outRearDrivingVelocityCmd", outRearDrivingVelocityCmd).doc("");
    addPort("outLeftSteeringPositionCmd", outLeftSteeringPositionCmd).doc("");
    addPort("outRightSteeringPositionCmd", outRightSteeringPositionCmd).doc("");
    addPort("outRearSteeringPositionCmd", outRearSteeringPositionCmd).doc("");
    addPort("outRobotBlocked", outRobotBlocked).doc("");
}
