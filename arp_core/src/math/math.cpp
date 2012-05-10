/*
 * math.cpp
 *
 *  Created on: 05 may 2011
 *      Author: MOU
 */

#include <math/math.hpp>


using namespace Eigen;
using namespace std;

namespace arp_math
{

Rotation2 betweenMinusPiAndPlusPi(const Rotation2 & rot)
{
    double angle = betweenMinusPiAndPlusPi(rot.angle());
    return Rotation2(angle);
}

double betweenMinusPiAndPlusPi(const double angle)
{
    double a = betweenZeroAndTwoPi(angle);
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
    double a = betweenZeroAndPi(angle);
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
    return fmod(fmod(angle, 2 * PI) + 4. * PI, 2 * PI);
}

double betweenZeroAndPi(const double angle)
{
    return fmod(fmod(angle, PI) + 2. * PI, PI);
}

double deg2rad(const double deg)
{
    return deg * (PI / 180);
}

double rad2deg(const double rad)
{
    return rad * (180 / PI);
}

Eigen::VectorXd bubbleSort(const Eigen::VectorXd & v)
{
    Eigen::VectorXd m = v;
    unsigned int n = m.size();
    if( n == 0 )
        return m;

    if( n == 1 )
        return m;

    bool no_change;
    do
    {
        no_change = true;
        for(unsigned int j = 0; j < n-1 ; j++)
        {
            if(m(j) > m(j+1))
            {
                double tmp = m(j+1);
                m(j+1) = m(j);
                m(j) = tmp;
                no_change = false;
            }
        }
    }while(!no_change);
    return m;
}


std::pair< Eigen::VectorXd, Eigen::VectorXi>  bubbleSortIndices(const Eigen::VectorXd & v)
                        {
    Eigen::VectorXd m = v;
    unsigned int n = m.size();
    if( n == 0 )
        return std::make_pair(m, Eigen::VectorXi(0) );

    if( n == 1 )
        return std::make_pair(m, Eigen::VectorXi::Zero(1));

    Eigen::VectorXi indices(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        indices(i) = i;
    }

    bool no_change;
    do
    {
        no_change = true;
        for(unsigned int j = 0; j < n-1 ; j++)
        {
            if(m(j) > m(j+1))
            {
                double tmp = m(j+1);
                m(j+1) = m(j);
                m(j) = tmp;
                int tmp_i = indices(j+1);
                indices(j+1) = indices(j);
                indices(j) = tmp_i;
                no_change = false;
            }
        }
    }while(!no_change);

    return std::make_pair(m, indices);
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

double smoothStep(const double x, const double startValue, const double startLimit, const double endValue,
        const double endLimit)
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

double firstDerivateLimitation(const double input, const double lastOutput, const double period, const double dmin,
        const double dmax)
{
    double output = 0;
    double derivate = 0;

    if (period > 0 && dmin < dmax)
    {
        //calcul de la derivée
        derivate = (input - lastOutput) / period;

        //filtrage
        if (derivate > fabs(dmax))
            output = lastOutput + fabs(dmax) * period;
        else if (derivate < -fabs(dmin))
            output = lastOutput - fabs(dmin) * period;
        else
            output = input;
    }
    else
    {
        output = lastOutput;
    }

    return output;
}

std::vector<Eigen::VectorXi> combinaisons( const Eigen::VectorXi & v, const unsigned int n )
{
    std::vector<Eigen::VectorXi> out;
    if(n < 1)
        return out;
    if(n == 1)
    {
        for(unsigned int i = 0 ; i < v.size() ; i++)
        {
            out.push_back( Eigen::VectorXi::Ones(1) * v(i) );
        }
        return out;
    }
    unsigned int m = v.size();
    if( m < n )
        return out;
    if( m == n )
    {
        out.push_back( v );
        return out;
    }

    for(unsigned int i = 0 ; i < m-n+1 ; i++)
    {
        std::vector<Eigen::VectorXi> combs = combinaisons( v.tail(m-i-1) , n-1);
        for( unsigned int k = 0 ; k < combs.size() ; k++)
        {
            Eigen::VectorXi d(combs[k].size() + 1);
            d.tail(combs[k].size()) = combs[k];
            d(0) = v(i);
            out.push_back( d );
        }
    }
    return out;
}

std::vector<Eigen::VectorXi> combinaisons( const unsigned int p, const unsigned int n )
{
    Eigen::VectorXi indices(p);
        for(unsigned int i = 0 ; i < p ; i++)
        {
            indices(i) = i;
        }
    return combinaisons(indices, n);
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
    return timespec2Double(delay);
}

//on utilise un long double pour le calcul de temps pour des raisons de précision numérique
long double getTime(void)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return timespec2Double(now);
}

void linesIntersection(const Vector2 & p1, const Vector2 & p2, const Vector2 & p3, const Vector2 & p4, const double & epsilon, Vector2 & result,
        bool & parralel,bool & colinear)
{
    Vector3 q1(p1(0),p1(1),0);
    Vector3 q2(p2(0),p2(1),0);
    Vector3 q3(p3(0),p3(1),0);
    Vector3 q4(p4(0),p4(1),0);

    Vector3 q1q2=q2-q1;
    Vector3 q3q4=q4-q3;
    Vector3 q3q2=q3-q2;

    //check colinear
    if (fabs(q1q2.cross(q3q4)(2))<=epsilon and fabs(q1q2.cross(q3q2)(2))<=epsilon)
        {
        parralel = true;
        colinear=true;
        return;
        }

    // check parralel
    if (fabs(q1q2.cross(q3q4)(2))<=epsilon)
    {
        parralel = true;
        colinear=false;
        return;
    }

    //code taken from http://flassari.is/2008/11/line-line-intersection-in-cplusplus/

    // Store the values for fast access and easy
    // equations-to-code conversion
    double x1 = p1(0), x2 = p2(0), x3 = p3(0), x4 = p4(0);
    double y1 = p1(1), y2 = p2(1), y3 = p3(1), y4 = p4(1);

    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

// Get the x and y
    double pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
    double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

// Return the point of intersection
    result(0) = x;
    result(1) = y;
    parralel = false;
    colinear=false;
    return;
}

double sign(double number)
{
if (number>=0.0)
        return 1.0;
    else
        return -1.0;

}

std::string toStrMaxDecimals(double value, int decimals)
{
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(decimals);
    ss << value;
    std::string s = ss.str();
    if(decimals > 0 && s[s.find_last_not_of('0')] == '.') {
        s.erase(s.size() - decimals + 1);
    }
    return s;
}



}

