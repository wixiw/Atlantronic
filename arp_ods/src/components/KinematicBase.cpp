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

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase)

KinematicBase::KinematicBase(const std::string& name) :
        OdsTaskContext(name), propMinSpeed(0.001), m_oldTime(0.0), propRobotBlockedTimeout(2.0)
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

}

void KinematicBase::run()
{
    Log(INFO) << ">> KinematicBase::run()   -----------------------------";

    double dt = getDt();

    //check robot blocked
    checkRobotBlocked();

    //filter the input command to get a reachable command that we are sure the hardware will be capable to do
    if (KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist, attrMotorsCurrentState, attrParams, dt,
            propMinSpeed, attrAcceptableTwist, attrQuality) == false)
    {
        LOG(Error) << "Failed to filter desired twist to an acceptable twist" << endlog();
    }

    //compute the motor commands to do the filtered twist command
    if (UbiquityKinematics::twist2Motors(attrAcceptableTwist, attrMotorsCurrentState, attrTurretState,
            attrMotorStateCommand, attrParams) == false)
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }


    Log(INFO) << "acceptable Twist=                                " << attrAcceptableTwist.toString() << " phi="
            << attrAcceptableTwist.speedAngle() << " v=" << attrAcceptableTwist.speedNorm();




    //gestion du cas pas de vitesse pour ne pas bouger les tourelles
    //TODO a supprimer avec le filtre ?
    if (attrAcceptableTwist.speedNorm() <= propMinSpeed)
    {

        attrMotorStateCommand.steering = attrMotorsCurrentState.steering;
    }
    Log(INFO) << "<< KinematicBase::run()";
}

double KinematicBase::getDt()
{
    double time = arp_math::getTime();
    if (m_oldTime == 0.0)
    {
        //first time here
        m_oldTime = time;
        return 0.01;
    }
    else
    {
        double dt = time - m_oldTime;
        m_oldTime = time;
        return dt;
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
    // distance bewteen twist. thetap is given a coefficient 0.04=0.2² to represent the speed at a 20cm lever
    double speederror = attrCurrentTwist.distanceTo(attrAcceptableTwist, Vector3(1.0, 1.0, 0.2));

    //////////////////////
    Log(INFO) << ">> KinematicBase::consistencyMeasuredvsCommanded()";
    Log(INFO) << "command  : "<<attrAcceptableTwist.toString();
    Log(INFO) << "measure  : "<<attrCurrentTwist.toString();
    Log(INFO) << "error    : "<<speederror;
    Log(INFO) << "<< KinematicBase::consistencyMeasuredvsCommanded()";
    ////////////////////


    // 20 mm/s difference accepted
    return speederror < 0.02;
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
    outRobotBlocked.write(attrRobotBlockedTimeout);

}

void KinematicBase::createOrocosInterface()
{
    addAttribute("attrTwistCmd", attrTwistCmd);
    addAttribute("attrCurrentTwist", attrCurrentTwist);
    addAttribute("attrAcceptableTwist", attrAcceptableTwist);
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorStateCommand", attrMotorStateCommand);
    addAttribute("attrMotorsCurrentState", attrMotorsCurrentState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrQuality", attrQuality);
    addAttribute("attrRobotBlockedTimeout", attrRobotBlockedTimeout);
    addAttribute("attrBlockTime", attrBlockTime);

    addProperty("propMinSpeed", propMinSpeed).doc("");
    addProperty("propRobotBlockedTimeout", propRobotBlockedTimeout).doc("");

    addPort("inTwistCmd", inTwistCmd).doc("");
    addPort("inCurrentTwist", inCurrentTwist).doc("");
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
