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

std::ostream& arp_math::operator<<(std::ostream& flux, arp_math::ICRSpeed const& t)
{
    return flux << t.toString();
}

ICRSpeed::ICRSpeed(double ro, double phi, double delta)
{
    m_ro = ro;
    m_ICR = ICR(phi, delta);
}

ICRSpeed::ICRSpeed(double ro, const ICR& ICR)
{
    m_ro = ro;
    m_ICR = ICR;
}

ICRSpeed::ICRSpeed(const Twist2DNorm& twist)
{
    initFromTwist(twist);

}

ICRSpeed::ICRSpeed(const Twist2D& twist)
{
    initFromTwist(Twist2DNorm(twist));
}

ICRSpeed::ICRSpeed(const ICRSpeed& icrSpeed)
{
    m_ro = icrSpeed.ro();
    m_ICR = ICR(icrSpeed.getICR());
}

void ICRSpeed::initFromTwist(const Twist2DNorm& twist)
{
    // if twist is null, then ICRspeed cannot be defined. default ICRSpeed is used
        if (fabs(twist.vx()) < vx_min and fabs(twist.vy()) < vy_min and fabs(twist.vh()) < vh_min)
        {
            m_ro = 0;
            m_ICR = ICR(0, 0);
            return;
        }

        //ro
        m_ro = sqrt(twist.vx() * twist.vx() + twist.vy() * twist.vy() + twist.vh() * twist.vh());
        m_ICR = ICR(twist.getTVector());

}

Twist2DNorm ICRSpeed::twistNorm() const
{
    double vx;
    double vy;
    double vh;

    vx = ro() * cos(delta()) * cos(phi());
    vy = ro() * cos(delta()) * sin(phi());
    vh = ro() * sin(delta());

    return Twist2DNorm(vx, vy, vh);
}

Twist2D ICRSpeed::twist() const
{
    return twistNorm().getTwist();
}

Vector3 ICRSpeed::speedDirection() const
{
    return m_ICR.getCartesianVector();
}

ICRSpeed ICRSpeed::getOppositeRep() const
{
    ICR antipodICR = m_ICR.getAntipodICR();
    return ICRSpeed(-m_ro, antipodICR.phi(), antipodICR.delta());
}

ICRSpeed ICRSpeed::getNormalizedRep() const
{
    if (m_ro<0)
        return getOppositeRep();
    else
        return ICRSpeed(*this);
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

ICR ICRSpeed::getICR() const
{
    return m_ICR;
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
    s << "(" << ro() << "m/s ," << rad2deg(phi()) << "° ," << rad2deg(delta()) << "° )";
    return s.str();
}

ICRSpeed ICRSpeed::createIdleFromICRPosition(const Vector2& ICRPosition)
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

ICRSpeed ICRSpeed::transport(const Pose2D & p) const
{
    ICRSpeed transportedSpeed;

    double a = cos(delta()) * cos(phi()) - p.y()/Twist2DNorm::dmax * sin(delta());
    double b = cos(delta()) * sin(phi()) + p.x()/Twist2DNorm::dmax * sin(delta());
    double c = sin(delta());

    double a2 = a*a;
    double b2 = b*b;
    double c2 = c*c;

    transportedSpeed.ro( ro() * sqrt(a2 + b2 + c2) );
    transportedSpeed.phi( atan2(b,a) - p.angle() );
    transportedSpeed.delta( atan(c/sqrt(a2 + b2)) );

    return transportedSpeed;
}

//TODO les coeff n'ont plus de sens, ils sont portés par position norm
double ICRSpeed::distanceTo(ICRSpeed other, double coefTrans, double coefRot) const
{
    return this->twist().distanceTo(other.twist(),coefTrans,coefRot);
}

double ICRSpeed::getTranslationSpeedNorm()
{
    return fabs(ro()*cos(delta()));
}

bool ICRSpeed::operator ==(const ICRSpeed& other) const
{
    if ( this->ro() == other.ro()
            && this->delta() == other.delta()
            && this->phi() == other.phi() )
    {
        return true;
    }

    ICRSpeed oppositeOther = other.getOppositeRep();
    if( this->ro() == oppositeOther.ro()
            && this->delta() == oppositeOther.delta()
            && this->phi() == oppositeOther.phi() )
    {
        return true;
    }

    return false;
}

bool ICRSpeed::operator !=(const ICRSpeed& other) const
{
    return ( ! (*this == other) );
}
