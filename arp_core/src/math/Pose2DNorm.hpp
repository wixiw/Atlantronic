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
    Pose2DNorm(Pose2D pose);

    Pose2D getPose();

        /* cette valeur permet de normaliser la rotation et de lui donner le meme ordre de grandeur que les translations sur le robot*/
        //TODO demande à willy ou ca va.    >>>>    Willy, ou ca va ? <<<<<<
    // arf zut en plus j'ai un doublon avec le twist2DNorm
        const static double dmax = 0.2;
};


std::ostream operator <<(std::ostream os, arp_math::Pose2D _pose);

}

#endif /* _ARP_MATH_POSE2DNORM_HPP_ */
