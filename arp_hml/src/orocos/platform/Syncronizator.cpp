/*
 * Syncronizator.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#include "Syncronizator.hpp"
#include <rtt/Component.hpp>

using namespace arp_core;
using namespace arp_model;
using namespace arp_hml;
using namespace Eigen;

ORO_LIST_COMPONENT_TYPE( arp_hml::Syncronizator )

Syncronizator::Syncronizator(const std::string name):
    HmlTaskContext(name)
{
    createOrocosInterface();
}

void Syncronizator::updateHook()
{
    outMotorMeasures.write(readInputs());
    inCanSync.readNewest(m_syncTime);
    outClock.write(m_syncTime);
}

MotorState Syncronizator::readInputs()
{
    MotorState motorState;
    inLeftDrivingPosition.readNewest(motorState.driving.left.position);
    inRightDrivingPosition.readNewest(motorState.driving.right.position);
    inRearDrivingPosition.readNewest(motorState.driving.rear.position);
    inLeftSteeringPosition.readNewest(motorState.steering.left.position);
    inRightSteeringPosition.readNewest(motorState.steering.right.position);
    inRearSteeringPosition.readNewest(motorState.steering.rear.position);

    inLeftDrivingVelocity.readNewest(motorState.driving.left.velocity);
    inRightDrivingVelocity.readNewest(motorState.driving.right.velocity);
    inRearDrivingVelocity.readNewest(motorState.driving.rear.velocity);
    inLeftSteeringVelocity.readNewest(motorState.steering.left.velocity);
    inRightSteeringVelocity.readNewest(motorState.steering.right.velocity);
    inRearSteeringVelocity.readNewest(motorState.steering.rear.velocity);

    inLeftDrivingTorque.readNewest(motorState.driving.left.torque);
    inRightDrivingTorque.readNewest(motorState.driving.right.torque);
    inRearDrivingTorque.readNewest(motorState.driving.rear.torque);
    inLeftSteeringTorque.readNewest(motorState.steering.left.torque);
    inRightSteeringTorque.readNewest(motorState.steering.right.torque);
    inRearSteeringTorque.readNewest(motorState.steering.rear.torque);

    return motorState;
}

void Syncronizator::createOrocosInterface()
{
    addPort("outClock",outClock)
                .doc("");
    addPort("outMotorMeasures",outMotorMeasures)
                .doc("");
    addPort("inCanSync",inCanSync)
            .doc("");

    addPort("inLeftDrivingVelocity",inLeftDrivingVelocity)
            .doc("");
    addPort("inRightDrivingVelocity",inRightDrivingVelocity)
            .doc("");
    addPort("inRearDrivingVelocity",inRearDrivingVelocity)
            .doc("");
    addPort("inLeftSteeringVelocity",inLeftSteeringVelocity)
            .doc("");
    addPort("inRightSteeringVelocity",inRightSteeringVelocity)
            .doc("");
    addPort("inRearSteeringVelocity",inRearSteeringVelocity)
            .doc("");

    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");
    addPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("");
    addPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("");
    addPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("");
    addPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("");

    addPort("inLeftDrivingTorque",inLeftDrivingTorque)
            .doc("");
    addPort("inRightDrivingTorque",inRightDrivingTorque)
            .doc("");
    addPort("inRearDrivingTorque",inRearDrivingTorque)
            .doc("");
    addPort("inLeftSteeringTorque",inLeftSteeringTorque)
            .doc("");
    addPort("inRightSteeringTorque",inRightSteeringTorque)
            .doc("");
    addPort("inRearSteeringTorque",inRearSteeringTorque)
            .doc("");
}
