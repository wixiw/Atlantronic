/*
 * Stats.cpp
 *
 *  Created on: 23 February 2012
 *      Author: Boris
 */

#include "Stats.hpp"
#include <exceptions/NotImplementedException.hpp>

using namespace Eigen;

namespace arp_math
{

double mean(const Eigen::VectorXd & v)
{
    if(v.size() == 0)
    {
        return 0.;
    }
    else
    {
        return v.sum() / v.size();
    }
}

double stddev(const Eigen::VectorXd & v)
{
    if(v.size() == 0)
    {
        return 0.;
    }
    else
    {
        double m = arp_math::mean(v);
        double s = 0.;
        for (unsigned int i = 0; i < v.size(); i++)
        {
            s += (m - v(i)) * (m - v(i));
        }
        return sqrt(s / v.size());
    }
}

double median(const Eigen::VectorXd & v)
{
    Eigen::VectorXd m = arp_math::bubbleSort(v);
    unsigned int n = m.size();
    if( n == 0 )
        return 0.;

    if( n == 1 )
        return m(0);

    return m((unsigned int)((n-1) / 2));
}

}
