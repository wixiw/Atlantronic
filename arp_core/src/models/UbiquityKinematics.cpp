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
    oTS.leftSteeringPosition = iMS.leftSteeringPosition*iParams.getTurretRatio()
            + iParams.getLeftTurretZero();
    oTS.rightSteeringPosition = iMS.rightSteeringPosition*iParams.getTurretRatio()
            + iParams.getRightTurretZero();
    oTS.rearSteeringPosition = iMS.rearSteeringPosition*iParams.getTurretRatio()
            + iParams.getRearTurretZero();

    oTS.leftDrivingVelocity = iParams.getLeftWheelDiameter()/2
            *(iMS.leftDrivingVelocity*iParams.getTractionRatio() + iMS.leftSteeringVelocity*iParams.getTurretRatio());
    oTS.rightDrivingVelocity = iParams.getRightWheelDiameter()/2
            *(iMS.rightDrivingVelocity*iParams.getTractionRatio() + iMS.rightSteeringVelocity*iParams.getTurretRatio());
    oTS.rearDrivingVelocity = iParams.getRearWheelDiameter()/2
            *(iMS.rearDrivingVelocity*iParams.getTractionRatio() + iMS.rearSteeringVelocity*iParams.getTurretRatio());

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
    oMS.leftSteeringPosition = (iTS.leftSteeringPosition - iParams.getLeftTurretZero())
            /iParams.getTurretRatio();
    oMS.rightSteeringPosition = (iTS.rightSteeringPosition - iParams.getRightTurretZero())
            /iParams.getTurretRatio();
    oMS.rearSteeringPosition = (iTS.rearSteeringPosition - iParams.getRearTurretZero())
            /iParams.getTurretRatio();

    oMS.leftDrivingVelocity = (2*iTS.leftDrivingVelocity/iParams.getLeftWheelDiameter() - iSMV.leftSteeringVelocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.rightDrivingVelocity = (2*iTS.rightDrivingVelocity/iParams.getRightWheelDiameter() - iSMV.rightSteeringVelocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.rearDrivingVelocity = (2*iTS.rearDrivingVelocity/iParams.getRearWheelDiameter() - iSMV.rearSteeringVelocity*iParams.getTurretRatio())
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
        A(0,3) = cos(iTS.leftSteeringPosition) * iTS.leftDrivingVelocity;
        A(1,3) = sin(iTS.leftSteeringPosition) * iTS.leftDrivingVelocity;
    }
    {
        A(2,0) =  iParams.getRightTurretPosition().y();
        A(3,0) = -iParams.getRightTurretPosition().x();
        A(2,1) = -1.;
        A(3,1) =  0.;
        A(2,2) =  0.;
        A(3,2) = -1.;
        A(2,3) = cos(iTS.rightSteeringPosition) * iTS.rightDrivingVelocity;
        A(3,3) = sin(iTS.rightSteeringPosition) * iTS.rightDrivingVelocity;
    }
    {
        A(4,0) =  iParams.getRearTurretPosition().y();
        A(5,0) = -iParams.getRearTurretPosition().x();
        A(4,1) = -1.;
        A(5,1) =  0.;
        A(4,2) =  0.;
        A(5,2) = -1.;
        A(4,3) = cos(iTS.rearSteeringPosition) * iTS.rearDrivingVelocity;
        A(5,3) = sin(iTS.rearSteeringPosition) * iTS.rearDrivingVelocity;
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
    oTS.leftSteeringPosition = Tleft.speedAngle();
    oTS.rightSteeringPosition = Tright.speedAngle();
    oTS.rearSteeringPosition = Trear.speedAngle();

    //recuperation des vitesses
    oTS.leftDrivingVelocity = Tleft.speedNorm();
    oTS.rightDrivingVelocity = Tright.speedNorm();
    oTS.rearDrivingVelocity = Trear.speedNorm();

    //normalisations
    normalizeDirection(oTS.leftSteeringPosition, oTS.leftDrivingVelocity);
    normalizeDirection(oTS.rightSteeringPosition, oTS.rightDrivingVelocity);
    normalizeDirection(oTS.rearSteeringPosition, oTS.rearDrivingVelocity);

    return true;
}

bool UbiquityKinematics::motors2Twist(const MotorState & iMS, arp_math::Twist2D& oTw, SlippageReport& oSR, const UbiquityParams & iParams)
{
    bool res = true;
    TurretState ioTS;
    res &= motors2Turrets(iMS, ioTS, iParams);
    res &= turrets2Twist(ioTS, oTw, oSR, iParams);
    return res;
}

bool UbiquityKinematics::twist2Motors(const arp_math::Twist2D & iTw, const SteeringMotorVelocities & iSMV, MotorState& oMS, const UbiquityParams & iParams)
{
    bool res = true;
    TurretState ioTS;
    res &= twist2Turrets(iTw, ioTS, iParams);
    res &= turrets2Motors(ioTS, iSMV, oMS, iParams);
    return res;
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

