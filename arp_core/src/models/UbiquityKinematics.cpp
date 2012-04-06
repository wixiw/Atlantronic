/*
 * UbiquityKinematics.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityKinematics.hpp"

#include "models/Logger.hpp"

#include <iostream>
#include <Eigen/SVD>
using namespace std;
using namespace arp_math;
using namespace arp_model;
using namespace arp_core::log;

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

    Eigen::Matrix<double, 6, 4> A;
    {
        A(0,0) =  iParams.getLeftTurretPosition().y();
        A(1,0) = -iParams.getLeftTurretPosition().x();
        A(0,1) = -1.;
        A(1,1) =  0.;
        A(0,2) =  0.;
        A(1,2) = -1.;
        A(0,3) = cos(iTS.leftSteeringTurretPosition) * iTS.leftDrivingTurretVelocity;
        A(1,3) = sin(iTS.leftSteeringTurretPosition) * iTS.leftDrivingTurretVelocity;
    }
    {
        A(2,0) =  iParams.getRightTurretPosition().y();
        A(3,0) = -iParams.getRightTurretPosition().x();
        A(2,1) = -1.;
        A(3,1) =  0.;
        A(2,2) =  0.;
        A(3,2) = -1.;
        A(2,3) = cos(iTS.rightSteeringTurretPosition) * iTS.rightDrivingTurretVelocity;
        A(3,3) = sin(iTS.rightSteeringTurretPosition) * iTS.rightDrivingTurretVelocity;
    }
    {
        A(4,0) =  iParams.getRearTurretPosition().y();
        A(5,0) = -iParams.getRearTurretPosition().x();
        A(4,1) = -1.;
        A(5,1) =  0.;
        A(4,2) =  0.;
        A(5,2) = -1.;
        A(4,3) = cos(iTS.rearSteeringTurretPosition) * iTS.rearDrivingTurretVelocity;
        A(5,3) = sin(iTS.rearSteeringTurretPosition) * iTS.rearDrivingTurretVelocity;
    }

    Eigen::VectorXd s = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
    Eigen::MatrixXd V = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV();

    oSR.kernelQuality = s(2) / s(3);

    Eigen::VectorXd X = V.col(3);
    X = X / X(3);

    oTw.vh( X(0) );
    oTw.vx( X(1) );
    oTw.vy( X(2) );

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

