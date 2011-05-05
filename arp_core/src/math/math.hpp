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
    const double RAD_S_TO_RPM = 30 / PI;
    /** Use this to multiply a value in RPM to get a value in rad/s */
    const double RPM_TO_RAD_S = PI / 30;

    /** Use this to multiply a value in rad to get a value in turn */
    const double RAD_TO_TURN = 1 / (2 * PI);
    /** Use this to multiply a value in turn to get a value in rad */
    const double TURN_TO_RAD = 2 * PI;

    /**
     * Return any angle in -pi +pi
     */
    double normalizeAngle(double angle);

    /**
     * Return "value" saturated to min or max
     */
    double saturate(double value, double min, double max);

    /**
     *  Return sqrt(value), but if value is negative return negative sqrt(value)
     */
    double sqrt2(double value);

    /**
     * Return f(x) = extrapolated value between startValue and endValue, but with respect to the position of "x" between startLimit and endLimit
     *
     * f(x)=
     *
     * end value       |            ______________
     *                 |          /
     * ----------------|---------/---------------------
     *                 |        /
     * startValue _____|_______/
     *                 |
     *                  startlimit endlimit
     *
     */
    double smoothStep(double x, double startValue, double startLimit,
            double endValue, double endLimit);
}

#endif /* _ARPMATH_MATH_HPP_ */
