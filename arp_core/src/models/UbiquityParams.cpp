/*
 * UbiquityParams.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityParams.hpp"

using namespace arp_core;

UbiquityParams::UbiquityParams():
        m_leftTurretPosition(0.000, 0.200, 0),
        m_rightTurretPosition(0.000, -0.200, 0),
        m_rearTurretPosition(-0.250, 0.000, 0),
        m_leftTurretZero(0),
        m_rightTurretZero(0),
        m_rearTurretZero(0),
        m_leftWheelDiameter(0.066),
        m_rightWheelDiameter(0.066),
        m_rearWheelDiameter(0.066),
        m_tractionRatio(1.0),
        m_turretRatio(0.25)
{
}

bool UbiquityParams::check() const
{
    if( m_leftWheelDiameter <= 0.010 || m_leftWheelDiameter >= 3 )
        return false;
    if( m_rightWheelDiameter <= 0.010 || m_rightWheelDiameter >= 3 )
        return false;
    if( m_rearWheelDiameter <= 0.010 || m_rearWheelDiameter >= 3 )
        return false;

    if( m_tractionRatio <= 0 )
        return false;
    if( m_turretRatio <= 0 )
        return false;

    if( m_leftTurretPosition == m_rightTurretPosition
            || m_leftTurretPosition == m_rearTurretPosition
            || m_rightTurretPosition == m_rearTurretPosition)
        return false;

    //tout va bien
    return true;
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

void UbiquityParams::setLeftTurretPosition(Pose2D leftTurretPosition)
{
    m_leftTurretPosition = leftTurretPosition;
}

void UbiquityParams::setRearTurretPosition(Pose2D rearTurretPosition)
{
    m_rearTurretPosition = rearTurretPosition;
}

void UbiquityParams::setRightTurretPosition(Pose2D rightTurretPosition)
{
    m_rightTurretPosition = rightTurretPosition;
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

void UbiquityParams::setTractionRatio(double tractionRatio)
{
    m_tractionRatio = tractionRatio;
}

void UbiquityParams::setTurretRatio(double turretRatio)
{
    m_turretRatio = turretRatio;
}







