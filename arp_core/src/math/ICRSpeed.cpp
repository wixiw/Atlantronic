/*
 * ICRSpeed.cpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include "math/ICRSpeed.hpp"
#include "math/ICR.hpp"
#include <iostream>

using namespace arp_math;
using namespace Eigen;
using namespace std;

std::ostream &operator<<(std::ostream &flux, arp_math::ICRSpeed const& t)
{
    return flux << t.toString();
}

ICRSpeed::ICRSpeed(double ro, double phi, double delta)
{
    m_ro = ro;
    m_ICR = ICR(phi, delta);
}

ICRSpeed::ICRSpeed(double ro, ICR ICR)
{
    m_ro = ro;
    m_ICR = ICR;
}

ICRSpeed::ICRSpeed(Twist2DNorm twist)
{
    initFromTwist(twist);

}

ICRSpeed::ICRSpeed(Twist2D twist)
{
    initFromTwist(Twist2DNorm(twist));
}

void ICRSpeed::initFromTwist(Twist2DNorm twist)
{
    // if twist is null, then ICRspeed cannot be defined. default ICRSpeed is used
        if (twist.vx() == 0.0 and twist.vy() == 0.0 and twist.vh() == 0.0)
        {
            m_ro = 0;
            m_ICR = ICR(0, 0);
            return;
        }

        //ro
        m_ro = sqrt(twist.vx() * twist.vx() + twist.vy() * twist.vy() + twist.vh() * twist.vh());
        m_ICR = ICR(twist.getTVector());

}

Twist2DNorm ICRSpeed::twistNorm()
{
    double vx;
    double vy;
    double vh;

    vx = ro() * cos(delta()) * cos(phi());
    vy = ro() * cos(delta()) * sin(phi());
    vh = ro() * sin(delta());

    return Twist2DNorm(vx, vy, vh);
}

Twist2D ICRSpeed::twist()
{
    return twistNorm().getTwist();
}

Vector3 ICRSpeed::speedDirection()
{
    return m_ICR.getCartesianVector();
}

ICRSpeed ICRSpeed::getOppositeRep()
{
    ICR antipodICR = m_ICR.getAntipodICR();
    return ICRSpeed(-m_ro, antipodICR.phi(), antipodICR.delta());
}

double ICRSpeed::ro() const
{
    return m_ro;
}

double ICRSpeed::phi() const
{
    return m_ICR.phi();
}

double ICRSpeed::delta() const
{
    return m_ICR.delta();
}

double& ICRSpeed::roRef()
{
    return m_ro;
}

double& ICRSpeed::phiRef()
{
    return m_ICR.phiRef();
}

double& ICRSpeed::deltaRef()
{
    return m_ICR.deltaRef();
}

void ICRSpeed::ro(double ro)
{
    m_ro = ro;
}

void ICRSpeed::phi(double phi)
{
    m_ICR.phi(phi);
}

void ICRSpeed::delta(double delta)
{
    m_ICR.delta(delta);
}

std::string ICRSpeed::toString() const
{
    std::ostringstream s;
    s << "(" << ro() << "," << phi() << "," << delta() << ")";
    return s.str();
}

ICRSpeed ICRSpeed::createIdleFromICRPosition(Vector2 ICRPosition)
{
    double phi = betweenMinusPiAndPlusPi(atan2(ICRPosition[1], ICRPosition[0])-PI/2);
    double delta = atan(Twist2DNorm::dmax/ICRPosition.norm());

    return ICRSpeed(0, phi, delta);
}

ICRSpeed ICRSpeed::createIdleFromTranslation(double angle)
{
    double phi = betweenMinusPiAndPlusPi(angle+PI/2);
    double delta = 0;

    return ICRSpeed(0, phi, delta);
}

