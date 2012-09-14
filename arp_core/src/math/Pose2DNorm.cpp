/*
 * Pose2D.cpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

#include "Pose2DNorm.hpp"
#include <math/MathFactory.hpp>
#include <iostream>


using namespace arp_math;

Pose2DNorm::Pose2DNorm(double _x, double _y, double _h)
: Pose2D(_x, _y, _h)
{
}

Pose2DNorm::Pose2DNorm(Pose2D pose)
{
    x(pose.x());
    y(pose.y());
    h(pose.h() * dmax);
}

Pose2D Pose2DNorm::getPose()
{
    return Pose2D(x(), y(), h() / dmax);
}


std::ostream& operator <<(std::ostream& os, Pose2DNorm _pose)
{
  os << _pose.toString();
  return os;
}

