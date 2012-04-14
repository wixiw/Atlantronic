/*
 * math.hpp
 *
 *  Created on: 23 avr. 2011
 *      Author: Boris
 */

#ifndef _ARPMATH_MATH_HPP_
#define _ARPMATH_MATH_HPP_

#define EIGEN_DONT_ALIGN
#define EIGEN_DONT_VECTORIZE
#include <Eigen/Core>
#include <Eigen/Geometry>

//a supprimer quand tout le mode aura switché a betweenMinusPiAndPlusPi
#define normalizeAngle betweenMinusPiAndPlusPi

namespace arp_math
{
    typedef Eigen::Vector2d Vector2;
    typedef Eigen::Vector3d Vector3;
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
     * TODO faire les tests unitaires !!!!!!!!!
     */

    double betweenMinusPiAndPlusPi( double angle);
    Rotation2 betweenMinusPiAndPlusPi(Rotation2 rot);
    double betweenZeroAndTwoPi( double angle);

    double betweenMinusPiAndPlusPi( double angle);
    double betweenZeroAndTwoPi( double angle);
    double betweenMinusPi2AndPlusPi2(double angle);
    double betweenZeroAndPi(double angle);

    double deg2rad(double deg);
    double rad2deg(double rad);

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
     * Limite la dérivée première d'une fonction
     */
    double firstDerivateLimitation(double input, double lastOutput, double period, double vmin, double vmax);

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
    long double delta_t(struct timespec begin, struct timespec now);

    /** return time **/
    long double getTime(void);
}

#endif /* _ARPMATH_MATH_HPP_ */
