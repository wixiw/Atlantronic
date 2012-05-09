/*
 * PolarSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "PolarSegment.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

PolarSegment::Params::Params()
: rangeThres(0.08)
{
}

std::string PolarSegment::Params::getInfo() const
{
    std::stringstream ss;
    ss << "PolarSegment params :" << std::endl;
    ss << " [*] rangeThres : " << rangeThres << " (m)" << std::endl;
    return ss.str();
}

bool PolarSegment::Params::checkConsistency() const
{
    if( rangeThres <= 0.)
    {
        Log( WARN ) << "PolarSegment::Params::checkConsistency" << " - " << "inconsistent parameters (rangeThres <= 0.)";
        return false;
    }
    return true;
}

std::vector<DetectedObject> PolarSegment::apply(const LaserScan & raw, const Params & p)
{
    std::vector<DetectedObject> out;
    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "PolarSegment::apply" << " - " << "Parameters are not consistent => Return 1-sized vector containing DetectedObject(raw)";
        out.push_back(DetectedObject(raw));
        return out;
    }

    if(raw.getSize() < 2 )
    {
        Log( NOTICE ) << "PolarSegment::apply" << " - " << "Less than two points in LaserScan => Return 1-sized vector containing DetectedObject(raw)";
        out.push_back(DetectedObject(raw));
        return out;
    }


    MatrixXd rawPolar = raw.getPolarData();
    bool b = raw.areCartesianDataAvailable();
    MatrixXd rawCartesian = raw.getCartesianData();
    unsigned int index = 0;
    double deltaTheta = (rawPolar.block(2,1,1,raw.getSize()-1) - rawPolar.block(2,0,1,raw.getSize()-1)).minCoeff();
    unsigned int N = raw.getSize();
    for(unsigned int i = 1 ; i < N ; i++)
    {
        if( abs(rawPolar(1,i) - rawPolar(1,i-1)) > p.rangeThres || rawPolar(2,i) - rawPolar(2,i-1) > 1.5*deltaTheta )
        {
            LaserScan ls;
            ls.setPolarData( rawPolar.block(0, index, 3, i-index) );
            if(b)
            {
                ls.computeCartesianData( rawCartesian.block(0, index, 5, i-index).row(0),
                                         rawCartesian.block(0, index, 5, i-index).row(3),
                                         rawCartesian.block(0, index, 5, i-index).row(4),
                                         rawCartesian.block(0, index, 5, i-index).row(5) );
            }
            index = i;
            out.push_back(DetectedObject(ls));
        }
    }
    LaserScan ls;
    ls.setPolarData( rawPolar.block(0, index, 3, N-index) );
    if(b)
    {
        ls.computeCartesianData( rawCartesian.row(0), rawCartesian.row(3), rawCartesian.row(4), rawCartesian.row(5) );
    }
    out.push_back(DetectedObject(ls));
    return out;
}
