/*
 * math.cpp
 *
 *  Created on: 05 may 2011
 *      Author: MOU
 */

#include <math/math.hpp>

namespace arp_math
{

Rotation2 betweenMinusPiAndPlusPi(const Rotation2 &     rot)
{
    double angle = betweenMinusPiAndPlusPi(rot.angle());
    return Rotation2(angle);
}

double betweenMinusPiAndPlusPi(const double angle)
{
    double a = betweenZeroAndTwoPi( angle );
    if (a > PI)
    {
        a = a - 2 * PI;
    }
    if (a <= -PI)
    {
        a = a + 2 * PI;
    }
    return a;
}

double betweenMinusPi2AndPlusPi2(const double angle)
{
    double a = betweenZeroAndPi( angle );
    if (a > M_PI_2)
    {
        a = a - PI;
    }
    if (a <= -M_PI_2)
    {
        a = a + PI;
    }
    return a;
}

double betweenZeroAndTwoPi(const double angle)
{
    return fmod( fmod(angle, 2 * PI) + 4. * PI, 2 * PI);
}

double betweenZeroAndPi(const double angle)
{
    return fmod( fmod(angle, PI) + 2. * PI, PI);
}

double deg2rad(const double deg)
{
    return deg * (PI / 180);
}

double rad2deg(const double rad)
{
    return rad * (180 / PI);
}

double saturate(const double value, const double min, const double max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

double sqrt2(const double value)
{
    if (value > 0)
        return sqrt(value);
    else
        return -sqrt(-value);
    return 0;
}

double smoothStep(const double x, const double startValue, const double startLimit, const double endValue, const double endLimit)
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

double firstDerivateLimitation(const double input, const double lastOutput, const double period, const double dmin, const double dmax)
{
    double output=0;
    double derivate=0;

    if( period > 0 && dmin < dmax)
    {
        //calcul de la derivée
        derivate = (input - lastOutput)/period;

        //filtrage
        if( derivate > fabs(dmax) )
            output = lastOutput + fabs(dmax)*period;
        else if( derivate < -fabs(dmin) )
            output = lastOutput -fabs(dmin)*period;
        else
            output = input;
    }
    else
    {
        output = lastOutput;
    }

    return output;
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

//on utilise un long double pour le calcul de temps pour des raisons de précision numérique
long double timespec2Double(const timespec & now)
{
    long double time = now.tv_sec + (long double) (now.tv_nsec) / 1E9;
    return time;
}

//on utilise un long double pour le calcul de temps pour des raisons de précision numérique
long double delta_t(struct timespec begin, struct timespec now)
{
    timespec delay;
    delta_t(&delay, begin, now);
    return  timespec2Double(delay);
}

//on utilise un long double pour le calcul de temps pour des raisons de précision numérique
long double getTime(void)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return timespec2Double(now);
}

}

