/*
 * UbiquityStates.hpp
 *
 *  Created on: April 6 2012
 *      Author: ard
 */

#ifndef UBIQUITYSTATES_HPP_
#define UBIQUITYSTATES_HPP_

namespace arp_model
{

struct SteeringMotorPositions
{
        double leftSteeringPosition;
        double rightSteeringPosition;
        double rearSteeringPosition;

        SteeringMotorPositions():
            leftSteeringPosition(0.0),
            rightSteeringPosition(0.0),
            rearSteeringPosition(0.0)
        {}
};

struct DrivingMotorPositions
{
        double leftDrivingPosition;
        double rightDrivingPosition;
        double rearDrivingPosition;

        DrivingMotorPositions():
            leftDrivingPosition(0.0),
            rightDrivingPosition(0.0),
            rearDrivingPosition(0.0)
        {}
};

struct SteeringMotorVelocities
{
        double leftSteeringVelocity;
        double rightSteeringVelocity;
        double rearSteeringVelocity;

        SteeringMotorVelocities():
            leftSteeringVelocity(0.0),
            rightSteeringVelocity(0.0),
            rearSteeringVelocity(0.0)
        {}
};

struct DrivingMotorVelocities
{
        double leftDrivingVelocity;
        double rightDrivingVelocity;
        double rearDrivingVelocity;

        DrivingMotorVelocities():
            leftDrivingVelocity(0.0),
            rightDrivingVelocity(0.0),
            rearDrivingVelocity(0.0)
        {}
};

struct SteeringMotorTorques
{
        double leftSteeringTorque;
        double rightSteeringTorque;
        double rearSteeringTorque;

        SteeringMotorTorques():
            leftSteeringTorque(0.0),
            rightSteeringTorque(0.0),
            rearSteeringTorque(0.0)
        {}
};

struct DrivingMotorTorques
{
        double leftDrivingTorque;
        double rightDrivingTorque;
        double rearDrivingTorque;

        DrivingMotorTorques():
            leftDrivingTorque(0.0),
            rightDrivingTorque(0.0),
            rearDrivingTorque(0.0)
        {}
};

struct MotorState
{
        double leftDrivingPosition;
        double rightDrivingPosition;
        double rearDrivingPosition;
        double leftSteeringPosition;
        double rightSteeringPosition;
        double rearSteeringPosition;
        double leftDrivingVelocity;
        double rightDrivingVelocity;
        double rearDrivingVelocity;
        double leftSteeringVelocity;
        double rightSteeringVelocity;
        double rearSteeringVelocity;
        double leftDrivingTorque;
        double rightDrivingTorque;
        double rearDrivingTorque;
        double leftSteeringTorque;
        double rightSteeringTorque;
        double rearSteeringTorque;

        MotorState():
            leftDrivingPosition(0.0),
            rightDrivingPosition(0.0),
            rearDrivingPosition(0.0),
            leftSteeringPosition(0.0),
            rightSteeringPosition(0.0),
            rearSteeringPosition(0.0),
            leftDrivingVelocity(0.0),
            rightDrivingVelocity(0.0),
            rearDrivingVelocity(0.0),
            leftSteeringVelocity(0.0),
            rightSteeringVelocity(0.0),
            rearSteeringVelocity(0.0),
            leftDrivingTorque(0.0),
            rightDrivingTorque(0.0),
            rearDrivingTorque(0.0),
            leftSteeringTorque(0.0),
            rightSteeringTorque(0.0),
            rearSteeringTorque(0.0)
        {}

        /*
        std::string toString() const
        {
            std::stringstream ss;
            ss << "Driving : "
                   << "(" << leftDrivingMotorPosition << ","
                   << rightDrivingMotorPosition << ","
                   << rearDrivingMotorPosition << ") m" << ","
                   << "(" << leftDrivingMotorVelocity << ","
                   << rightDrivingMotorVelocity << ","
                   << rearDrivingMotorVelocity << ") m/s" << ","
                   << "(" << leftDrivingMotorTorque << ","
                   << rightDrivingMotorTorque << ","
                   << rearDrivingMotorTorque << ") Nm" << " - "
               << "Steering : "
                   << "(" << leftSteeringMotorPosition << ","
                   << rightSteeringMotorPosition << ","
                   << rearSteeringMotorPosition << ") rad" << ","
                   << "(" << leftSteeringMotorVelocity << ","
                   << rightSteeringMotorVelocity << ","
                   << rearSteeringMotorVelocity << ") rad/s" << ","
                   << "(" << leftSteeringMotorTorque << ","
                   << rightSteeringMotorTorque << ","
                   << rearSteeringMotorTorque << ") Nm";
            return ss.str();
        }*/
};



struct SteeringTurretPositions
{
        double leftSteeringPosition;
        double rightSteeringPosition;
        double rearSteeringPosition;

        SteeringTurretPositions():
            leftSteeringPosition(0.0),
            rightSteeringPosition(0.0),
            rearSteeringPosition(0.0)
        {}
};


struct DrivingTurretPositions
{
        double leftDrivingPosition;
        double rightDrivingPosition;
        double rearDrivingPosition;

        DrivingTurretPositions():
            leftDrivingPosition(0.0),
            rightDrivingPosition(0.0),
            rearDrivingPosition(0.0)
        {}
};


struct SteeringTurretVelocities
{
        double leftSteeringVelocity;
        double rightSteeringVelocity;
        double rearSteeringVelocity;

        SteeringTurretVelocities():
            leftSteeringVelocity(0.0),
            rightSteeringVelocity(0.0),
            rearSteeringVelocity(0.0)
        {}
};

struct DrivingTurretVelocities
{
        double leftDrivingVelocity;
        double rightDrivingVelocity;
        double rearDrivingVelocity;

        DrivingTurretVelocities():
            leftDrivingVelocity(0.0),
            rightDrivingVelocity(0.0),
            rearDrivingVelocity(0.0)
        {}
};

struct SteeringTurretTorques
{
        double leftSteeringTorque;
        double rightSteeringTorque;
        double rearSteeringTorque;

        SteeringTurretTorques():
            leftSteeringTorque(0.0),
            rightSteeringTorque(0.0),
            rearSteeringTorque(0.0)
        {}
};

struct DrivingTurretTorques
{
        double leftDrivingTorque;
        double rightDrivingTorque;
        double rearDrivingTorque;

        DrivingTurretTorques():
            leftDrivingTorque(0.0),
            rightDrivingTorque(0.0),
            rearDrivingTorque(0.0)
        {}
};

struct TurretState
{
        double leftDrivingPosition;
        double rightDrivingPosition;
        double rearDrivingPosition;
        double leftSteeringPosition;
        double rightSteeringPosition;
        double rearSteeringPosition;
        double leftDrivingVelocity;
        double rightDrivingVelocity;
        double rearDrivingVelocity;
        double leftSteeringVelocity;
        double rightSteeringVelocity;
        double rearSteeringVelocity;
        double leftDrivingTorque;
        double rightDrivingTorque;
        double rearDrivingTorque;
        double leftSteeringTorque;
        double rightSteeringTorque;
        double rearSteeringTorque;

        TurretState():
            leftDrivingPosition(0.0),
            rightDrivingPosition(0.0),
            rearDrivingPosition(0.0),
            leftSteeringPosition(0.0),
            rightSteeringPosition(0.0),
            rearSteeringPosition(0.0),
            leftDrivingVelocity(0.0),
            rightDrivingVelocity(0.0),
            rearDrivingVelocity(0.0),
            leftSteeringVelocity(0.0),
            rightSteeringVelocity(0.0),
            rearSteeringVelocity(0.0),
            leftDrivingTorque(0.0),
            rightDrivingTorque(0.0),
            rearDrivingTorque(0.0),
            leftSteeringTorque(0.0),
            rightSteeringTorque(0.0),
            rearSteeringTorque(0.0)

        {}

        /*
        std::string toString() const
        {
            std::stringstream ss;
            ss << "Driving : " << "(" << leftDrivingTurretPosition << ","
               << rightDrivingTurretPosition << ","
               << rearDrivingTurretPosition << ") m" << ","
               << "(" << leftDrivingTurretVelocity << ","
               << rightDrivingTurretVelocity << ","
               << rearDrivingTurretVelocity << ") m/s" << " - "
               << "Steering : " << "(" << leftSteeringTurretPosition << ","
               << rightSteeringTurretPosition << ","
               << rearSteeringTurretPosition << ") rad" << ","
               << "(" << leftSteeringTurretVelocity << ","
               << rightSteeringTurretVelocity << ","
               << rearSteeringTurretVelocity << ") rad/s";
            return ss.str();
        }*/
};


struct SlippageReport
{
        double kernelQuality;
};
} /* namespace arp_model */
#endif /* UBIQUITYSTATES_HPP_ */
