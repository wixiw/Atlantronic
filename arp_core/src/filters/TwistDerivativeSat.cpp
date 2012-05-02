/*
 * TwistDerivativeSat.cpp
 *
 *  Created on: 5 Mai 2012
 *      Author: Boris
 */


#include <filters/TwistDerivativeSat.hpp>

using namespace arp_math;

arp_math::Twist2D TwistDerivativeSat::apply(arp_math::Twist2D targetTwist, arp_math::Twist2D prevTwist, arp_math::Vector3 limits, double period)
{
    arp_math::Twist2D out = targetTwist;
    out.vx( firstDerivateLimitation(targetTwist.vx(), prevTwist.vx(), period, -fabs(limits[0]), fabs(limits[0])) );
    out.vy( firstDerivateLimitation(targetTwist.vy(), prevTwist.vy(), period, -fabs(limits[1]), fabs(limits[1])) );
    out.vh( firstDerivateLimitation(targetTwist.vh(), prevTwist.vh(), period, -fabs(limits[2]), fabs(limits[2])) );
    return out;
}
