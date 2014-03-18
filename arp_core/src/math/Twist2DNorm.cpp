/*
 * Twist2D.cpp
 *
 *  Created on: 7 sept. 2010
 *      Author: romain
 */

#include <math/Twist2DNorm.hpp>
#include <math/Twist2D.hpp>
#include <math/MathFactory.hpp>

using namespace arp_math;

const double Twist2DNorm::dmax = Pose2DNorm::dmax;

std::ostream &operator<<(std::ostream &flux, arp_math::Twist2DNorm const& t)
{
    return flux << t.toString();
}

Twist2DNorm::Twist2DNorm(double _vx, double _vy, double _vh) :
        Twist2D(_vx, _vy, _vh)
{
}

Twist2DNorm::Twist2DNorm(const Twist2D& twist)
{
    vx(twist.vx());
    vy(twist.vy());
    vh(twist.vh() * dmax);
}

Twist2D Twist2DNorm::getTwist() const
{
    return (Twist2D(vx(), vy(), vh() / dmax));
}
