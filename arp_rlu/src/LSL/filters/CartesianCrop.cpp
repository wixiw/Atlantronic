/*
 * CartesianCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CartesianCrop.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

CartesianCrop::Params::Params()
: minX(-1.5)
, maxX(1.5)
, minY(-1.)
, maxY(1.)
{
}

std::string CartesianCrop::Params::getInfo()
{
    std::stringstream ss;
    ss << "CartesianCrop params :" << std::endl;
    ss << " [*] minX : " << minX << " (m)" << std::endl;
    ss << " [*] maxX : " << maxX << " (m)" << std::endl;
    ss << " [*] minY : " << minY << " (m)" << std::endl;
    ss << " [*] maxY : " << maxY << " (m)" << std::endl;
    return ss.str();
}

bool CartesianCrop::Params::checkConsistency() const
{
    if( maxX < minX )
    {
        Log( NOTICE ) << "CartesianCrop::Params::checkConsistency" << " - " << "inconsistent parameters (maxX < minX)";
        return false;
    }
    if( maxY < minY )
    {
        Log( NOTICE ) << "CartesianCrop::Params::checkConsistency" << " - " << "inconsistent parameters (maxY < minY)";
        return false;
    }
    return true;
}

LaserScan CartesianCrop::apply(const LaserScan & raw, const Params & p)
{
    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "CartesianCrop::apply" << " - " << "Parameters are not consistent => Return raw LaserScan";
        return raw;
    }

    if( !raw.areCartesianDataAvailable() )
    {
        Log( ERROR ) << "CartesianCrop::apply" << " - " << "cartesian data are not available => Return raw LaserScan";
        return raw;
    }

    MatrixXd rawPolarData = raw.getPolarData();
    MatrixXd rawCartesianData = raw.getCartesianData();
    unsigned int N = 0;
    for(int i = 0 ; i < rawCartesianData.cols() ; i++)
    {
        if( rawCartesianData(1,i) <= p.maxX && p.minX <= rawCartesianData(1,i)
                && rawCartesianData(2,i) <= p.maxY && p.minY <= rawCartesianData(2,i) )
        {
            N++;
        }
    }

    MatrixXd newCartesianData = MatrixXd::Zero(6,N);
    MatrixXd newPolarData = MatrixXd::Zero(3,N);
    unsigned int k = 0;
    for (int i = 0; i < rawCartesianData.cols() ; i++)
    {
        if( rawCartesianData(1,i) <= p.maxX && p.minX <= rawCartesianData(1,i)
                && rawCartesianData(2,i) <= p.maxY && p.minY <= rawCartesianData(2,i) )
        {
            newCartesianData.col(k) = rawCartesianData.col(i);
            newPolarData.col(k) = rawPolarData.col(i);
            k++;
        }
    }

    LaserScan out;
    out.setPolarData(newPolarData);
    out.computeCartesianData(newCartesianData.row(0), newCartesianData.row(3), newCartesianData.row(4), newCartesianData.row(5));
    return out;
}
