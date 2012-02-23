/*
 * PolarSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "PolarSegment.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

PolarSegment::Params::Params()
: rangeThres(0.08)
{
}

std::string PolarSegment::Params::getInfo()
{
    std::stringstream ss;
    ss << "PolarSegment params :" << std::endl;
    ss << " [*] rangeThres : " << rangeThres << " (m)" << std::endl;
    return ss.str();
}

bool PolarSegment::Params::checkConsistency() const
{
    if( rangeThres <= 0.)
        return false;
    return true;
}

std::vector<LaserScan> PolarSegment::apply(const LaserScan & raw, const Params & p)
{
    std::vector<LaserScan> out;
    if( !p.checkConsistency() )
    {
        out.push_back(raw);
        return out;
    }

    if(raw.getSize() < 2 )
    {
        out.push_back(raw);
        return out;
    }


    MatrixXd rawPolar = raw.getPolarData();
    bool b = raw.areCartesianDataAvailable();
    MatrixXd rawCartesian = raw.getCartesianData();
    int index = 0;
    for(int i = 1 ; i < raw.getSize() ; i++)
    {
        if( abs(rawPolar(1,i) - rawPolar(1,i-1)) > p.rangeThres )
        {
            LaserScan ls;
            ls.setPolarData( rawPolar.block(0, index, 3, i-index) );
            index = i;
            if(b)
            {
                ls.computeCartesianData( rawCartesian.row(0), rawCartesian.row(3), rawCartesian.row(4), rawCartesian.row(5) );
            }
            out.push_back(ls);
        }
    }
    LaserScan ls;
    ls.setPolarData( rawPolar.block(0, index, 3, raw.getSize()-index) );
    if(b)
    {
        ls.computeCartesianData( rawCartesian.row(0), rawCartesian.row(3), rawCartesian.row(4), rawCartesian.row(5) );
    }
    out.push_back(ls);
    return out;
}
