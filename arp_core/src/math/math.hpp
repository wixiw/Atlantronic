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

#include <vector>

namespace arp_math
{
    typedef Eigen::Vector2d Vector2;
    typedef Eigen::Vector3d Vector3;
    typedef Eigen::Rotation2D<double> Rotation2;
    typedef Eigen::Matrix<double,3,3> Covariance3;

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

    double betweenMinusPiAndPlusPi( const double angle);
    Rotation2 betweenMinusPiAndPlusPi(const Rotation2 & rot);
    double betweenZeroAndTwoPi( const double angle);

    double betweenMinusPiAndPlusPi( const double angle);
    double betweenZeroAndTwoPi( const double angle);
    double betweenMinusPi2AndPlusPi2(const double angle);
    double betweenZeroAndPi(const double angle);

    double deg2rad(const double deg);
    double rad2deg(const double rad);
    double sign(double number);

    /*
     * return the angle between 2 3D vectors
     */
    double angleBetweenVectors(Vector3 M1, Vector3 M2);

    /**
     * Tri à bulle
     * \return le vecteur d'entrée trié par ordre croissant
     */
    Eigen::VectorXd bubbleSort(const Eigen::VectorXd &);

    /**
     * Tri à bulle qui renvoie aussi le vecteur des indices
     * \return le vecteur d'entrée trié par ordre croissant
     */
    std::pair< Eigen::VectorXd, Eigen::VectorXi > bubbleSortIndices(const Eigen::VectorXd &);

    /**
     * Return "value" saturated to min or max
     */
    double saturate(const double value, const double min, const double max);

    /**
     *  Return sqrt(value), but if value is negative return negative sqrt(value)
     */
    double sqrt2(const double value);

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
    double smoothStep(const double x, const double startValue, const double startLimit,
            const double endValue, const double endLimit);

    /**
     * Limite la dérivée première d'une fonction
     */
    double firstDerivateLimitation(const double input, const double lastOutput, const double period, const double dmin, const double dmax);

    /**
     * absolute value for floats
     */
    double d_abs(const double x);

    /**
     * Permet d'énumérer les combinaisons (au sens de la combinatoire)de n éléments parmi le vecteur v
     */
    std::vector<Eigen::VectorXi> combinaisons( const Eigen::VectorXi & v, const unsigned int n );

    /**
     * Permet d'énumérer les combinaisons (au sens de la combinatoire)de n éléments parmi le vecteur v
     */
    std::vector<Eigen::VectorXi> combinaisons( const unsigned int p, const unsigned int n );

    /**
     * transforme un time spec en double
     */
    long double timespec2Double(const timespec &);
    timespec double2Timespec(long double now);

    /**
     * Elapsed time between begin and now, using data type timespec.
     * Return values simply to indicate return point
     */
    void delta_t(struct timespec *interval, struct timespec begin,
            struct timespec now);
    long double delta_t(struct timespec begin, struct timespec now);

    /**
     * Increment the time yourself by a step
     */
    void incrementTime(struct timespec& time, long double delta);

    /** return time **/
    long double getTime(void);

    /** computes the point of intersection of 2 lines
     * @param[in] p1 :first point of first line
     * @param[in] p2 :second point of first line
     * @param[in] p3 :first point of second line
     * @param[in] p4 :second point of second line
     * @param[out] return :intersection point if it exist
     * @param[out] parralel: there is no intersection point, lines are parralel
     */
    void linesIntersection(const Vector2 & p1,const Vector2 & p2,const Vector2 & p3,const Vector2 & p4, const double & epsilon, Vector2 & result, bool & parralel,bool & colinear);

    std::string toStrMaxDecimals(double value, int decimals);
}

#endif /* _ARPMATH_MATH_HPP_ */
