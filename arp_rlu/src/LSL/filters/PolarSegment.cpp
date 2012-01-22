/*
 * PolarSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "PolarSegment.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

PolarSegment::Params::Params()
{
}

std::string PolarSegment::Params::getInfo()
{
    std::stringstream ss;
    ss << "PolarSegment params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan PolarSegment::apply(const LaserScan & raw, const Params & p)
{
    LaserScan out = raw;
    return out;
}
