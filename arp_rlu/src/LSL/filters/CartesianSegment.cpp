/*
 * CartesianSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CartesianSegment.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CartesianSegment::Params::Params()
{
}

std::string CartesianSegment::Params::getInfo()
{
    std::stringstream ss;
    ss << "CartesianSegment params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan CartesianSegment::apply(const LaserScan & raw, const Params & p)
{
    LaserScan out = raw;
    return out;
}
