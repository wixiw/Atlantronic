/*
 * Pose2D.hpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

// TODO BMO :
// * utiliser les références et les const
// * opérateurs approximation et différence
// * faire apparaitre cette todolist dans le Doxygen
#ifndef _ARP_MATH_POSE2DNORM_HPP_
#define _ARP_MATH_POSE2DNORM_HPP_

#include <math/math.hpp>
#include <math/Pose2D.hpp>

namespace arp_math
{

class Pose2DNorm: public Pose2D
{
    public:
        Pose2DNorm(double _x = 0., double _y = 0., double _h = 0.);
        Pose2DNorm(const Pose2D& pose);

        std::string toString() const;
        Pose2D getPose() const;

        const static double dmax = 0.2;

};

std::ostream operator <<(std::ostream os, arp_math::Pose2D _pose);

}

#endif /* _ARP_MATH_POSE2DNORM_HPP_ */
