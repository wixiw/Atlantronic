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
#include "Geometry.hpp"

namespace arp_math
{
    typedef Eigen::Matrix<double, 2, 1> Vector2;
    typedef Eigen::Matrix<double, 3, 1> Vector3;
    typedef Eigen::Rotation2D<double> Rotation2;

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
    Rotation2 normalizeAngle(Rotation2 rot);

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

    /**
     * absolute value for floats
     */
    double d_abs(double x);

    /**
     * Elapsed time between begin and now, using data type timespec.
     * Return values simply to indicate return point
     */
    void delta_t(struct timespec *interval, struct timespec begin,
            struct timespec now);
    double delta_t(struct timespec begin, struct timespec now);
}

#endif /* _ARPMATH_MATH_HPP_ */
