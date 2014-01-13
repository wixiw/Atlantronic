/*
 * Twist2DNorm.hpp
 *
 *  Created on: 7 september 2012
 *      Author: Romain
 */

#ifndef _ARP_MATH_TWIST2DNORM_HPP_
#define _ARP_MATH_TWIST2DNORM_HPP_

#include <math/math.hpp>
#include <math/Pose2DNorm.hpp>
#include <math/Twist2D.hpp>
#include <iostream>

namespace arp_math
{
/*
 * \nonstableyet
 *
 * \class Twist2DNorm
 *
 * \brief Torseur cinématique du robot, dont la rotation est normalisée en la multipliant par une distance
 *
 */

class Twist2DNorm: public Twist2D
{
    public:
        Twist2DNorm(double _vx = 0, double _vy = 0, double _vh = 0);
        Twist2DNorm(const Twist2D& twist);

        Twist2D getTwist() const;

        static double dmax;
};

std::ostream &operator<<( std::ostream &flux, arp_math::Twist2DNorm const& t);

}

#endif /* _ARP_MATH_TWIST2DNORM_HPP_ */
