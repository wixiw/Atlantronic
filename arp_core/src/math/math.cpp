/*
 * math.cpp
 *
 *  Created on: 05 may 2011
 *      Author: MOU
 */

#include "math.hpp"

namespace arp_math
{

Rotation2 betweenMinusPiAndPlusPi(Rotation2 rot)
{
    double angle = betweenMinusPiAndPlusPi(rot.angle());
    return Rotation2(angle);
}

double betweenMinusPiAndPlusPi(double angle)
{
    angle = betweenZeroAndTwoPi( angle );
    if (angle > PI)
    {
        angle = angle - 2 * PI;
    }
    if (angle < -PI)
    {
        angle = angle + 2 * PI;
    }
    return angle;
}

double betweenZeroAndTwoPi(double angle)
{
    return fmod( fmod(angle, 2 * PI) + 4. * PI, 2 * PI);
}

double deg2rad(double deg)
{
    return deg * (PI / 180);
}

double rad2deg(double rad)
{
    return rad * (180 / PI);
}

double saturate(double value, double min, double max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

double sqrt2(double value)
{
    if (value > 0)
        return sqrt(value);
    if (value < 0)
        return -sqrt(-value);
    return 0;
}

double smoothStep(double x, double startValue, double startLimit, double endValue, double endLimit)
{
    //il s'agit d'une fonction toute simple qui permet de trouver la valeur intermediaire entre deux valeurs corresponsdant à des modes de calculs différents

    if (x <= startLimit)
    {
        return startValue;
    }
    if (x > startLimit && x < endLimit)
    {
        //le barycentre
        return (startValue * (endLimit - x) + endValue * (x - startLimit)) / (endLimit - startLimit);
    }
    if (x >= endLimit)
    {
        return endValue;
    }

    // ne devrait jamais arriver
    return -666;

}

void delta_t(struct timespec *interval, struct timespec begin, struct timespec now)
{
    interval->tv_nsec = now.tv_nsec - begin.tv_nsec; /* Subtract 'decimal fraction' first */
    if (interval->tv_nsec < 0)
    {
        interval->tv_nsec += 1000000000; /* Borrow 1sec from 'tv_sec' if subtraction -ve */
        interval->tv_sec = now.tv_sec - begin.tv_sec - 1; /* Subtract whole number of seconds and return 1 */
    }
    else
    {
        interval->tv_sec = now.tv_sec - begin.tv_sec; /* Subtract whole number of seconds and return 0 */
    }
}

double delta_t(struct timespec begin, struct timespec now)
{
    timespec delay;
    delta_t(&delay, begin, now);
    return delay.tv_sec + (double) (delay.tv_nsec) / 1E9;
}

double getTime(void)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec + (double) (now.tv_nsec) / 1E9;
}

}

