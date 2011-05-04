/*
 * math.hpp
 *
 *  Created on: 23 avr. 2011
 *      Author: Boris
 */

#ifndef _ARPMATH_MATH_HPP_
#define _ARPMATH_MATH_HPP_

#define EIGEN_DONT_ALIGN
#include <Eigen/Core>

namespace arp_math
{
    const double PI = std::acos(-1.0);

    /** Use this to multiply a value in rad/s to get a value in RPM */
    const double RAD_S_TO_RPM = 30/PI;
    /** Use this to multiply a value in RPM to get a value in rad/s */
    const double RPM_TO_RAD_S = PI/30;

    /** Use this to multiply a value in rad to get a value in turn */
    const double RAD_TO_TURN= 1/(2*PI);
    /** Use this to multiply a value in turn to get a value in rad */
    const double TURN_TO_RAD = 2*PI;
}


#endif /* _ARPMATH_MATH_HPP_ */
