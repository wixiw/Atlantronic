/*
 * MedianFilter.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "MedianFilter.hpp"

#include "LSL/Logger.hpp"


using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

MedianFilter::Params::Params()
: width(3)
{
}

std::string MedianFilter::Params::getInfo() const
{
    std::stringstream ss;
    ss << "MedianFilter params :" << std::endl;
    ss << " [*] width: " << width << std::endl;
    return ss.str();
}

bool MedianFilter::Params::checkConsistency() const
{
    return true;
}

LaserScan MedianFilter::apply(const LaserScan & raw, const Params & p)
{
    MatrixXd rawData = raw.getPolarData();
    if( rawData.cols() == 0 )
    {
        Log( NOTICE ) << "MedianFilter::apply" << " - " << "LaserScan is empty => Return raw LaserScan";
        return LaserScan();
    }

    if( p.width == 0 )
    {
        Log( NOTICE ) << "MedianFilter::apply" << " - " << "p.width == 0 => Return raw LaserScan";
        return raw;
    }

    MatrixXd filtScan = raw.getPolarData();

    int infIndex = (int)((p.width-1)/2);
    int supIndex = (int)(p.width-1-(p.width-1)/2);
    for(int j = 0; j < rawData.cols() ; j++)
    {
        if( j-infIndex < 0 )
            continue;

        if( j+supIndex > rawData.cols() - 1 )
            continue;

        VectorXd v(p.width);
        for(unsigned int k = 0; k < p.width ; k++)
            v(k) = rawData(1,j-infIndex+k);
        filtScan(1,j) = arp_math::median(v);
    }

    LaserScan out = raw;
    out.setPolarData(filtScan);
    return out;
}

