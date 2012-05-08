/*
 * ICRSpeed.cpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include "math/ICRSpeed.hpp"
#include <iostream>

using namespace arp_math;
using namespace Eigen;
using namespace std;

std::ostream &operator<<(std::ostream &flux, arp_math::ICRSpeed const& t)
{
    return flux << t.toString();
}

ICRSpeed::ICRSpeed(double ro, double alpha, double q) :
        m_ro(ro), m_alpha(alpha), m_q(q)
{
}

ICRSpeed::ICRSpeed(Twist2D twist)
{
    double v;
    double ro;
    double alpha;
    double q;
    v = sqrt(twist.vx() * twist.vx() + twist.vy() * twist.vy());

    if (twist.vh() != 0.0)
        ro = atan(v / twist.vh());
    else
        ro = PI / 2.0;
    if (twist.vx() != 0.0 or twist.vy() != 0.0)
        alpha = atan2(twist.vy(), twist.vx());
    else
        alpha = 0.0;
    q = v + twist.vh();

    m_ro = ro;
    m_alpha = alpha;
    m_q = q;

}

Twist2D ICRSpeed::twist()
{
    double v;
    double vx;
    double vy;
    double vh;

    if (m_ro == PI / 2.0)
        v = m_q;
    else if (m_ro == 0.0)
        v = 0.0;
    else
        v = m_q / (1 + 1 / tan(m_ro));

    vx = v * cos(m_alpha);
    vy = v * sin(m_alpha);
    vh = m_q - v;

    return Twist2D(vx, vy, vh);
}

ICRSpeed ICRSpeed::createFromICR(Vector2 ICR,Vector2 speedPoint, Vector2 speed)
{
    //used for cross products
    Vector3 speed3(speed(0),speed(1),0);
    Vector3 ICR3(ICR(0),ICR(1),0);
    Vector3 speedPoint3(speedPoint(0),speedPoint(1),0);
    Vector3 ICR_to_speedpoint3=speedPoint3-ICR3;

    ICRSpeed result;

    //omega: we have
    // speed = v(speedPoint) = v(ICR)  +   omega  ^ ( ICR to speedpoint )
    // speed=omega  ^ ( ICR to speedpoint )
    // boris says that consequently
    // omega = ( ICR to speedpoint ) ^ speed / ( ICR to speedpoint )²
    Vector3 omega= ICR_to_speedpoint3.cross(speed3) / (ICR_to_speedpoint3.norm()*ICR_to_speedpoint3.norm());
    Vector3 speedRef=omega.cross(-1.0*ICR3);

    result.ro(atan(ICR.norm()*sign(omega(2))));

    if (ICR(1)!=0.0 or ICR(0)!=0.0)
        result.alpha(atan2(speedRef(1),speedRef(0)));
    else
        result.alpha(0.0);

    result.q(omega(2)+speedRef.norm());

    return result;
}

ICRSpeed ICRSpeed::createFromTranslation(double alpha,double speed)
{
    Twist2D twist(speed*cos(alpha),speed*sin(alpha),0.0);
    return ICRSpeed(twist);
}


double ICRSpeed::ro() const
{
    return m_ro;
}

double ICRSpeed::alpha() const
{
    return m_alpha;
}

double ICRSpeed::q() const
{
    return m_q;
}

double& ICRSpeed::roRef()
{
    return m_ro;
}

double& ICRSpeed::alphaRef()
{
    return m_alpha;
}

double& ICRSpeed::qRef()
{
    return m_q;
}

void ICRSpeed::ro(double ro)
{
    m_ro = ro;
}

void ICRSpeed::alpha(double alpha)
{
    m_alpha = alpha;
}

void ICRSpeed::q(double q)
{
    m_q = q;
}

std::string ICRSpeed::toString() const
{
    std::ostringstream s;
    s << "(" << ro() << "," << alpha() << "," << q() << ")";
    return s.str();
}
