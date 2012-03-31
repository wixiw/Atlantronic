/*
 * UbiquityParams.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityParams.hpp"

using namespace arp_core;

UbiquityParams::UbiquityParams()
{
    // TODO Auto-generated constructor stub

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



