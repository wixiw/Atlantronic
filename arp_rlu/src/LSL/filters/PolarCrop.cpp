/*
 * PolarCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "PolarCrop.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

PolarCrop::Params::Params()
: ParamsInterface()
, minRange(0.1)
, maxRange(10.0)
, minTheta(-PI)
, maxTheta(PI)
{
}

std::string PolarCrop::Params::getInfo() const
{
    std::stringstream ss;
    ss << "PolarCrop params :" << std::endl;
    ss << " [*] minRange : " << minRange << " (m)" << std::endl;
    ss << " [*] maxRange : " << maxRange << " (m)" << std::endl;
    ss << " [*] minTheta : " << rad2deg(minTheta) << " (deg)"<< std::endl;
    ss << " [*] maxTheta : " << rad2deg(maxTheta) << " (deg)"<< std::endl;
    return ss.str();
}

bool PolarCrop::Params::checkConsistency() const
{
    if( minRange < 0.)
    {
        Log( WARN ) << "PolarCrop::Params::checkConsistency" << " - " << "inconsistent parameters (minRange < 0.)";
        return false;
    }
    if( maxRange < 0.)
    {
        Log( WARN ) << "PolarCrop::Params::checkConsistency" << " - " << "inconsistent parameters (maxRange < 0.)";
        return false;
    }
    if( minRange > maxRange)
    {
        Log( WARN ) << "PolarCrop::Params::checkConsistency" << " - " << "inconsistent parameters (minRange > maxRange)";
        return false;
    }
    if( minTheta > maxTheta )
    {
        Log( WARN ) << "PolarCrop::Params::checkConsistency" << " - " << "inconsistent parameters ( minTheta > maxTheta )";
        return false;
    }
    return true;
}

LaserScan PolarCrop::apply(const LaserScan & raw, const Params & p)
{
    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "PolarCrop::apply" << " - " << "Parameters are not consistent => Return raw LaserScan";
        return raw;
    }

    MatrixXd rawData = raw.getPolarData();
    if( rawData.cols() == 0 )
    {
        Log( NOTICE ) << "PolarCrop::apply" << " - " << "LaserScan is empty => Return raw LaserScan";
        return raw;
    }

    unsigned int N = 0;
    for(int i = 0 ; i < rawData.cols() ; i++)
    {
        if( rawData(1,i) <= p.maxRange
         && p.minRange  <= rawData(1,i)
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
        if( rawData(1,i) <= p.maxRange
         && p.minRange  <= rawData(1,i)
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
