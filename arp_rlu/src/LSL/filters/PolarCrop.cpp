/*
 * PolarCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "PolarCrop.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

PolarCrop::Params::Params()
: ParamsInterface()
, minRange(0.1 * VectorXd::Ones(1))
, maxRange(10.0 * VectorXd::Ones(1))
, minTheta(-PI)
, maxTheta(PI)
{
}

std::string PolarCrop::Params::getInfo()
{
    std::stringstream ss;
    ss << "PolarCrop params :" << std::endl;
    ss << " [*] minRange size     : " << minRange.size() << std::endl;
    ss << " [*] minRange minCoeff : " << minRange.minCoeff() << " (m)"<< std::endl;
    ss << " [*] minRange maxCoeff : " << minRange.maxCoeff() << " (m)"<< std::endl;
    ss << " [*] maxRange size     : " << maxRange.size() << std::endl;
    ss << " [*] maxRange minCoeff : " << maxRange.minCoeff() << " (m)"<< std::endl;
    ss << " [*] maxRange maxCoeff : " << maxRange.maxCoeff() << " (m)"<< std::endl;
    ss << " [*] minTheta : " << rad2deg(minTheta) << " (deg)"<< std::endl;
    ss << " [*] maxTheta : " << rad2deg(maxTheta) << " (deg)"<< std::endl;
    return ss.str();
}

bool PolarCrop::Params::checkConsistency() const
{
    if( minRange.size() == 0 )
        return false;
    if( maxRange.size() == 0 )
        return false;
    if( minRange.minCoeff() < 0.)
        return false;
    if( maxRange.minCoeff() < 0.)
        return false;
    if( maxRange.size() > 1 )
    {
        if( minRange.size() > 1 )
        {
            if( (maxRange - minRange).minCoeff() < 0.)
                return false;
        }
        else
        {
            if( maxRange.minCoeff() - minRange[0] < 0.)
                return false;
        }
    }
    else
    {
        if( minRange.size() > 1 )
        {
            if( maxRange[0] - minRange.minCoeff() < 0.)
                return false;
        }
        else
        {
            if( maxRange[0] - minRange[0] < 0.)
                return false;
        }
    }
    if( minTheta > maxTheta )
        return false;
    return true;
}

LaserScan PolarCrop::apply(const LaserScan & raw, const Params & p)
{
    if( !p.checkConsistency() )
    {
        return raw;
    }

    MatrixXd rawData = raw.getPolarData();
    if( rawData.cols() == 0 )
    {
        return raw;
    }

    if( p.maxRange.size() == 0 )
    {
        return raw;
    }

    if( p.minRange.size() == 0 )
    {
        return raw;
    }

    VectorXd maxRanges = p.maxRange;
    VectorXd minRanges = p.minRange;
    if( p.maxRange.size() == 1 )
    {
        maxRanges = VectorXd::Ones(rawData.cols()) * p.maxRange(0);
    }

    if( p.maxRange.size() == 1 )
    {
        minRanges = VectorXd::Ones(rawData.cols()) * p.minRange(0);
    }

    if( maxRanges.size() != rawData.cols() )
    {
        return raw;
    }

    for(int i = 0 ; i < rawData.cols() ; i++)
    {
        if( maxRanges(i) < minRanges(i) )
        {
            return raw;
        }
    }

    if( minRanges.size() != rawData.cols() )
    {
        return raw;
    }

    unsigned int N = 0;
    for(int i = 0 ; i < rawData.cols() ; i++)
    {
        if( rawData(1,i) <= maxRanges(i)
         && minRanges(i) <= rawData(1,i)
         && rawData(2,i) <= p.maxTheta
         && p.minTheta   <= rawData(2,i))
        {
            N++;
        }
    }

    MatrixXd newData = MatrixXd::Zero(3,N);
    unsigned int k = 0;
    for (int i = 0; i < rawData.cols() ; i++)
    {
        if( rawData(1,i) <= maxRanges(i)
         && minRanges(i) <= rawData(1,i)
         && rawData(2,i) <= p.maxTheta
         && p.minTheta   <= rawData(2,i) )
        {
            newData.col(k) = rawData.col(i);
            k++;
        }
    }

    LaserScan out;
    out.setPolarData(newData);
    return out;
}
