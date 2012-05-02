/*
 * TwistDerivativeSat.hpp
 *
 *  Created on: 5 Mai 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_TWISTDERIVATIVESAT_HPP_
#define _ARP_MATH_TWISTDERIVATIVESAT_HPP_

#include <math/core>

namespace arp_math
{

/*
 * \nonstableyet
 *
 * \class TwistDerivativeSat
 *
 * \brief Saturation du la différence finie (première dérivée) entre 2 Twists
 */

class TwistDerivativeSat
{
    public:
        /**
         * @param[in] targetTwist est le Twist que l'on voudrait atteindre
         * @param[in] prevTwist est le Twist précédent
         * @param[in] limits est un Vector3 donnant les valeurs absolues des limites de la première dérivée sur chaque axe
         * @param[in] period le temps écoulé entre currentTwist et prevTwist.
         * @return le Twist saturé
         */
        static arp_math::Twist2D apply(arp_math::Twist2D targetTwist, arp_math::Twist2D prevTwist, arp_math::Vector3 limits, double period);
};

}


#endif /* _ARP_MATH_TWISTDERIVATIVESAT_HPP_ */
