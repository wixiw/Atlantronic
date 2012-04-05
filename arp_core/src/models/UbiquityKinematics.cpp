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
using namespace arp_math;

UbiquityKinematics::UbiquityKinematics()
{
    // TODO Auto-generated constructor stub

}

bool UbiquityKinematics::motors2Turrets(const MotorState & iMS,
        TurretState& oTS,
        const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
        return false;
    }

    //TODO modulo ? je pense pas
    oTS.leftSteeringTurretPosition = iMS.leftSteeringMotorPosition*iParams.getTurretRatio()
            + iParams.getLeftTurretZero();
    oTS.rightSteeringTurretPosition = iMS.rightSteeringMotorPosition*iParams.getTurretRatio()
            + iParams.getRightTurretZero();
    oTS.rearSteeringTurretPosition = iMS.rearSteeringMotorPosition*iParams.getTurretRatio()
            + iParams.getRearTurretZero();

    oTS.leftDrivingTurretVelocity = iParams.getLeftWheelDiameter()/2
            *(iMS.leftDrivingMotorVelocity*iParams.getTractionRatio() + iMS.leftSteeringMotorVelocity*iParams.getTurretRatio());
    oTS.rightDrivingTurretVelocity = iParams.getRightWheelDiameter()/2
            *(iMS.rightDrivingMotorVelocity*iParams.getTractionRatio() + iMS.rightSteeringMotorVelocity*iParams.getTurretRatio());
    oTS.rearDrivingTurretVelocity = iParams.getRearWheelDiameter()/2
            *(iMS.rearDrivingMotorVelocity*iParams.getTractionRatio() + iMS.rearSteeringMotorVelocity*iParams.getTurretRatio());

    return true;
}

bool UbiquityKinematics::turrets2Motors(const TurretState & iTS,
        const SteeringMotorVelocities & iSMV,
        MotorState& oMS,
        const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
        return false;
    }

    //TODO modulo ? je pense pas
    oMS.leftSteeringMotorPosition = (iTS.leftSteeringTurretPosition - iParams.getLeftTurretZero())
            /iParams.getTurretRatio();
    oMS.rightSteeringMotorPosition = (iTS.rightSteeringTurretPosition - iParams.getRightTurretZero())
            /iParams.getTurretRatio();
    oMS.rearSteeringMotorPosition = (iTS.rearSteeringTurretPosition - iParams.getRearTurretZero())
            /iParams.getTurretRatio();

    oMS.leftDrivingMotorVelocity = (2*iTS.leftDrivingTurretVelocity/iParams.getLeftWheelDiameter() - iSMV.leftSteeringMotorVelocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.rightDrivingMotorVelocity = (2*iTS.rightDrivingTurretVelocity/iParams.getRightWheelDiameter() - iSMV.rightSteeringMotorVelocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.rearDrivingMotorVelocity = (2*iTS.rearDrivingTurretVelocity/iParams.getRearWheelDiameter() - iSMV.rearSteeringMotorVelocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();

    return true;
}

bool UbiquityKinematics::turrets2Twist(const TurretState & iTS, Twist2D& oTw, SlippageReport& oSR, const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
        return false;
    }


    return true;
}

bool UbiquityKinematics::twist2Turrets(const Twist2D & iTw, TurretState& oTS, const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
        return false;
    }
    //WARNING ! On fait quoi avec le sens ?

    //calcul des Twist de la BR sur chaque tourelle
    Twist2D Tleft = iTw.transport(iParams.getLeftTurretPosition());
    Twist2D Tright = iTw.transport(iParams.getRightTurretPosition());
    Twist2D Trear = iTw.transport(iParams.getRearTurretPosition());

    //recuperation des angles
    oTS.leftSteeringTurretPosition = Tleft.speedAngle();
    oTS.rightSteeringTurretPosition = Tright.speedAngle();
    oTS.rearSteeringTurretPosition = Trear.speedAngle();

    //recuperation des vitesses
    oTS.leftDrivingTurretVelocity = Tleft.speedNorm();
    oTS.rightDrivingTurretVelocity = Tright.speedNorm();
    oTS.rearDrivingTurretVelocity = Trear.speedNorm();

    //normalisations
    normalizeDirection(oTS.leftSteeringTurretPosition, oTS.leftDrivingTurretVelocity);
    normalizeDirection(oTS.rightSteeringTurretPosition, oTS.rightDrivingTurretVelocity);
    normalizeDirection(oTS.rearSteeringTurretPosition, oTS.rearDrivingTurretVelocity);

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

