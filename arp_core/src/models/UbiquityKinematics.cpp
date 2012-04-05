/*
 * UbiquityKinematics.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityKinematics.hpp"
#include <iostream>
using namespace std;
using namespace arp_core;

UbiquityKinematics::UbiquityKinematics()
{
    // TODO Auto-generated constructor stub

}

bool UbiquityKinematics::motors2Turrets(MotorCommands const inputs,
        TurretCommands& outputs,
        CouplingSpeeds const turretSpeeds,
        UbiquityParams const params)
{
    if( !params.check() )
    {
        return false;
    }

    //TODO modulo ? je pense pas
    outputs.leftSteeringTurretPosition = inputs.leftSteeringMotorPosition*params.getTurretRatio()
            + params.getLeftTurretZero();
    outputs.rightSteeringTurretPosition = inputs.rightSteeringMotorPosition*params.getTurretRatio()
            + params.getRightTurretZero();
    outputs.rearSteeringTurretPosition = inputs.rearSteeringMotorPosition*params.getTurretRatio()
            + params.getRearTurretZero();

    outputs.leftDrivingTurretSpeed = params.getLeftWheelDiameter()/2
            *(inputs.leftDrivingMotorSpeed*params.getTractionRatio() + turretSpeeds.leftSteeringMotorSpeed*params.getTurretRatio());
    outputs.rightDrivingTurretSpeed = params.getRightWheelDiameter()/2
            *(inputs.rightDrivingMotorSpeed*params.getTractionRatio() + turretSpeeds.rightSteeringMotorSpeed*params.getTurretRatio());
    outputs.rearDrivingTurretSpeed = params.getRearWheelDiameter()/2
            *(inputs.rearDrivingMotorSpeed*params.getTractionRatio() + turretSpeeds.rearSteeringMotorSpeed*params.getTurretRatio());

    return true;
}

bool UbiquityKinematics::turrets2Motors(TurretCommands const inputs,
        MotorCommands& outputs,
        CouplingSpeeds const turretSpeeds,
        UbiquityParams const params)
{
    if( !params.check() )
    {
        return false;
    }

    //TODO modulo ? je pense pas
    outputs.leftSteeringMotorPosition = (inputs.leftSteeringTurretPosition - params.getLeftTurretZero())
            /params.getTurretRatio();
    outputs.rightSteeringMotorPosition = (inputs.rightSteeringTurretPosition - params.getRightTurretZero())
            /params.getTurretRatio();
    outputs.rearSteeringMotorPosition = (inputs.rearSteeringTurretPosition - params.getRearTurretZero())
            /params.getTurretRatio();

    outputs.leftDrivingMotorSpeed = (2*inputs.leftDrivingTurretSpeed/params.getLeftWheelDiameter() - turretSpeeds.leftSteeringMotorSpeed*params.getTurretRatio())
            /params.getTractionRatio();
    outputs.rightDrivingMotorSpeed = (2*inputs.rightDrivingTurretSpeed/params.getRightWheelDiameter() - turretSpeeds.rightSteeringMotorSpeed*params.getTurretRatio())
            /params.getTractionRatio();
    outputs.rearDrivingMotorSpeed = (2*inputs.rearDrivingTurretSpeed/params.getRearWheelDiameter() - turretSpeeds.rearSteeringMotorSpeed*params.getTurretRatio())
            /params.getTractionRatio();

    return true;
}

bool UbiquityKinematics::turrets2Twist(TurretCommands const inputs, Twist2D& outputs, Slippage& splippage, UbiquityParams const params)
{
    if( !params.check() )
    {
        return false;
    }


    return true;
}

bool UbiquityKinematics::twist2Turrets(Twist2D const  inputs, TurretCommands& outputs, UbiquityParams const params)
{
    if( !params.check() )
    {
        return false;
    }
    //WARNING ! On fait quoi avec le sens ?

    //calcul des Twist de la BR sur chaque tourelle
    Twist2D Tleft = inputs.transport(params.getLeftTurretPosition());
    Twist2D Tright = inputs.transport(params.getRightTurretPosition());
    Twist2D Trear = inputs.transport(params.getRearTurretPosition());

    //recuperation des angles
    outputs.leftSteeringTurretPosition = Tleft.speedAngle();
    outputs.rightSteeringTurretPosition = Tright.speedAngle();
    outputs.rearSteeringTurretPosition = Trear.speedAngle();

    //recuperation des vitesses
    outputs.leftDrivingTurretSpeed = Tleft.speedNorm();
    outputs.rightDrivingTurretSpeed = Tright.speedNorm();
    outputs.rearDrivingTurretSpeed = Trear.speedNorm();

    //normalisations
    normalizeDirection(outputs.leftSteeringTurretPosition, outputs.leftDrivingTurretSpeed);
    normalizeDirection(outputs.rightSteeringTurretPosition, outputs.rightDrivingTurretSpeed);
    normalizeDirection(outputs.rearSteeringTurretPosition, outputs.rearDrivingTurretSpeed);

    return true;
}

void UbiquityKinematics::normalizeDirection(double& angle, double& speed)
{
    //modulo 2PI pour commencer
    angle = betweenMinusPiAndPlusPi(angle);

    if( angle <= -M_PI_2 || angle > M_PI_2 )
    {
        if( angle < 0 )
            angle += M_PI;
        else
            angle -= M_PI;
        speed *= -1;
    }
}

