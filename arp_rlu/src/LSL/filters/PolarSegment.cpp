/*
 * PolarSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

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

bool PolarSegment::Params::checkConsistency()
{
    throw NotImplementedException();
    return false;
}

std::vector<LaserScan> PolarSegment::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    std::vector<LaserScan> out;
    return out;
}
