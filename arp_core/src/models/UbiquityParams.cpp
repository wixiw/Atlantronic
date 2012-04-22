/*
 * UbiquityParams.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityParams.hpp"
#include "models/Logger.hpp"

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace std;

UbiquityParams::UbiquityParams():
        m_leftTurretPosition(0.000, 0.155, 0),
        m_rightTurretPosition(0.000, -0.155, 0),
        m_rearTurretPosition(-0.175, 0.000, 0),
        m_chassisCenter(-0.058, 0.000, 0),
        m_leftTurretZero(0),
        m_rightTurretZero(0),
        m_rearTurretZero(0),
        m_leftWheelDiameter(0.066),
        m_rightWheelDiameter(0.066),
        m_rearWheelDiameter(0.066),
        m_tractionRatio(1.0),
        m_turretRatio(0.25),
        m_maxDrivingSpeed(1),
        m_maxDrivingAcc(1),
        m_maxDrivingTorque(-1),
        m_maxSteeringSpeed(125),
        m_maxSteeringAcc(52),
        m_maxSteeringTorque(-1)
{
}

bool UbiquityParams::check() const
{
    bool res = true;

    //on vérifie que les roues ont une dimension réaliste
    if( m_leftWheelDiameter <= 0.010 || m_leftWheelDiameter >= 3 )
    {
        Log( ERROR ) << "Left wheel diameter is not correct";
        res = false;
    }
    if( m_rightWheelDiameter <= 0.010 || m_rightWheelDiameter >= 3 )
    {
        Log( ERROR ) << "Right wheel diameter is not correct";
        res = false;
    }
    if( m_rearWheelDiameter <= 0.010 || m_rearWheelDiameter >= 3 )
    {
        Log( ERROR ) << "Rear wheel diameter is not correct";
        res = false;
    }

    //pas de rapport d'engrages négatifs ou nuls
    if( m_tractionRatio <= 0 )
    {
        Log( ERROR ) << "Traction gears ratio is too little";
        res = false;
    }
    if( m_turretRatio <= 0 )
    {
        Log( ERROR ) << "Turret gears ratio is too little";
        res = false;
    }

    //on verifie que les 2 tourelles ne sont pas confondues 2 à 2
    if( m_leftTurretPosition == m_rightTurretPosition
            || m_leftTurretPosition == m_rearTurretPosition
            || m_rightTurretPosition == m_rearTurretPosition)
    {
        Log( ERROR ) << "The 3 turrets are not distincts";
        res = false;
    }

    //on verifie que les vitesses max de traction/direction ont du sens
    if( m_maxDrivingSpeed <= 0.1 || m_maxDrivingSpeed >= 10 || m_maxSteeringSpeed <= 1 || m_maxSteeringSpeed >= 300)
    {
        Log( ERROR ) << "Max speeds incorrect";
        res = false;
    }

    //on verifie que les accelerations de traction on du sens
    //TODO à remplir quand on les aura
//    if( m_maxDrivingAcc <= 0 || m_maxDrivingAcc >= 15 || m_maxDrivingDec <= 0 || m_maxDrivingDec >= 15)
//    {
//        cerr << "Max drivign acc incorect" << endl;
//        return false;
//    }

    //acc ne peut pas etre negatif
    if( m_maxSteeringAcc <= 0 )
    {
        Log( ERROR ) << "Max steering acc incorect";
        res = false;
    }

    //pour le moment on a pas de modele dynamique on a pas de raison de les utiliser.
    if( m_maxDrivingTorque != -1 || m_maxSteeringTorque != -1 )
    {
        Log( ERROR ) << "Max torque incorect";
        res = false;
    }

    return res;
}


double UbiquityParams::getLeftTurretZero() const
{
    return m_leftTurretZero;
}

double UbiquityParams::getRearTurretZero() const
{
    return m_rearTurretZero;
}

double UbiquityParams::getRightTurretZero() const
{
    return m_rightTurretZero;
}

Pose2D UbiquityParams::getLeftTurretPosition() const
{
    return m_leftTurretPosition;
}

Pose2D UbiquityParams::getRearTurretPosition() const
{
    return m_rearTurretPosition;
}

Pose2D UbiquityParams::getRightTurretPosition() const
{
    return m_rightTurretPosition;
}

double UbiquityParams::getLeftWheelDiameter() const
{
    return m_leftWheelDiameter;
}

double UbiquityParams::getRearWheelDiameter() const
{
    return m_rearWheelDiameter;
}

double UbiquityParams::getRightWheelDiameter() const
{
    return m_rightWheelDiameter;
}

double UbiquityParams::getTractionRatio() const
{
    return m_tractionRatio;
}

double UbiquityParams::getTurretRatio() const
{
    return m_turretRatio;
}

double UbiquityParams::getMaxDrivingAcc() const
{
    return m_maxDrivingAcc;
}

double UbiquityParams::getMaxDrivingSpeed() const
{
    return m_maxDrivingSpeed;
}

double UbiquityParams::getMaxDrivingTorque() const
{
    return m_maxDrivingTorque;
}

double UbiquityParams::getMaxSteeringAcc() const
{
    return m_maxSteeringAcc;
}


double UbiquityParams::getMaxSteeringSpeed() const
{
    return m_maxSteeringSpeed;
}

double UbiquityParams::getMaxSteeringTorque() const
{
    return m_maxSteeringTorque;
}

double UbiquityParams::getMaxSteeringMotorAcc() const
{
    return  m_maxSteeringAcc/m_turretRatio;
}
double UbiquityParams::getMaxSteeringMotorSpeed() const
{
    return m_maxSteeringSpeed/m_turretRatio;
}
double UbiquityParams::getMaxDrivingMotorAcc() const
{
    return m_maxDrivingAcc/m_tractionRatio/(m_leftWheelDiameter+m_rightWheelDiameter+m_rearWheelDiameter)/6;
}
double UbiquityParams::getMaxDrivingMotorSpeed() const
{
    return m_maxDrivingSpeed/m_tractionRatio/(m_leftWheelDiameter+m_rightWheelDiameter+m_rearWheelDiameter)/6;
}



Pose2D UbiquityParams::getChassisCenter() const
{
    return m_chassisCenter;
}





double& UbiquityParams::getLeftTurretZeroRef()
{
    return m_leftTurretZero;
}

double& UbiquityParams::getRearTurretZeroRef()
{
    return m_rearTurretZero;
}

double& UbiquityParams::getRightTurretZeroRef()
{
    return m_rightTurretZero;
}

Pose2D& UbiquityParams::getLeftTurretPositionRef()
{
    return m_leftTurretPosition;
}

Pose2D& UbiquityParams::getRearTurretPositionRef()
{
    return m_rearTurretPosition;
}

Pose2D& UbiquityParams::getRightTurretPositionRef()
{
    return m_rightTurretPosition;
}

double& UbiquityParams::getLeftWheelDiameterRef()
{
    return m_leftWheelDiameter;
}

double& UbiquityParams::getRearWheelDiameterRef()
{
    return m_rearWheelDiameter;
}

double& UbiquityParams::getRightWheelDiameterRef()
{
    return m_rightWheelDiameter;
}

double& UbiquityParams::getTractionRatioRef()
{
    return m_tractionRatio;
}

double& UbiquityParams::getTurretRatioRef()
{
    return m_turretRatio;
}

double& UbiquityParams::getMaxDrivingAccRef()
{
    return m_maxDrivingAcc;
}


double& UbiquityParams::getMaxDrivingSpeedRef()
{
    return m_maxDrivingSpeed;
}

double& UbiquityParams::getMaxDrivingTorqueRef()
{
    return m_maxDrivingTorque;
}

double& UbiquityParams::getMaxSteeringAccRef()
{
    return m_maxSteeringAcc;
}

double& UbiquityParams::getMaxSteeringSpeedRef()
{
    return m_maxSteeringSpeed;
}

double& UbiquityParams::getMaxSteeringTorqueRef()
{
    return m_maxSteeringTorque;
}

Pose2D& UbiquityParams::getChassisCenterRef()
{
    return m_chassisCenter;
}









void UbiquityParams::setLeftTurretZero(double leftTurretZero)
{
    m_leftTurretZero = leftTurretZero;
}

void UbiquityParams::setRearTurretZero(double rearTurretZero)
{
    m_rearTurretZero = rearTurretZero;
}

void UbiquityParams::setRightTurretZero(double rightTurretZero)
{
    m_rightTurretZero = rightTurretZero;
}

void UbiquityParams::setLeftWheelDiameter(double leftWheelDiameter)
{
    m_leftWheelDiameter = leftWheelDiameter;
}

void UbiquityParams::setRearWheelDiameter(double rearWheelDiameter)
{
    m_rearWheelDiameter = rearWheelDiameter;
}

void UbiquityParams::setRightWheelDiameter(double rightWheelDiameter)
{
    m_rightWheelDiameter = rightWheelDiameter;
}










