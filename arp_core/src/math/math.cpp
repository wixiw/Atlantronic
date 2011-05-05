/*
 * math.cpp
 *
 *  Created on: 05 may 2011
 *      Author: MOU
 */

#include "math.hpp"

namespace arp_math
{

    double normalizeAngle(double angle)
    {
        angle = fmod(angle, 2 * PI);
        if (angle > PI)
            angle = angle - 2 * PI;
        if (angle < -PI)
            angle = angle + 2 * PI;
        return angle;
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

    double smoothStep(double x, double startValue,
            double startLimit, double endValue, double endLimit)
    {
        //il s'agit d'une fonction toute simple qui permet de trouver la valeur intermediaire entre deux valeurs corresponsdant à des modes de calculs différents

        if (x <= startLimit)
        {
            return startValue;
        }
        if (x > startLimit && x < endLimit)
        {
            //le barycentre
            return (startValue * (endLimit - x) + endValue * (x - startLimit))
                    / (endLimit - startLimit);
        }
        if (x >= endLimit)
        {
            return endValue;
        }

        return 0;

    }

}
