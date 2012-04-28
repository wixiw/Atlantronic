/*
 * ICRSpeed.cpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include "math/ICRSpeed.hpp"
#include <iostream>

using namespace arp_math;

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
    if (twist.vx() != 0 or twist.vy() != 0)
        alpha = atan2(twist.vy(), twist.vx());
    else
        alpha = PI / 2.0;
    q = v + twist.vh();

    m_ro = ro;
    m_alpha = alpha;
    m_q = q;

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

std::string ICRSpeed::toString() const
{
    std::ostringstream s;
    s << "(" << ro() << "," << alpha() << "," << q() << ")";
    return s.str();
}
