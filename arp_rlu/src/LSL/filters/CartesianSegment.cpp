/*
 * CartesianSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CartesianSegment.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CartesianSegment::Params::Params()
: kmeansMaxIterations(10)
, kmeansDispThres(0.01)
, minNbPoints(5)
, maxStddev(0.1)
{
}

std::string CartesianSegment::Params::getInfo()
{
    std::stringstream ss;
    ss << "CartesianSegment params :" << std::endl;
    ss << " [*] kmeansMaxIterations : " << kmeansMaxIterations << std::endl;
    ss << " [*] kmeansDispThres     : " << kmeansDispThres << " (m)"<< std::endl;
    ss << " [*] minNbPoints         : " << minNbPoints << std::endl;
    ss << " [*] maxStddev           : " << maxStddev << " (m)"<< std::endl;
    return ss.str();
}

bool CartesianSegment::Params::checkConsistency() const
{
    throw NotImplementedException();
    return false;
}

std::vector<LaserScan> CartesianSegment::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    std::vector<LaserScan> out;
    return out;
}


std::pair<LaserScan, LaserScan> CartesianSegment::kMeans(const LaserScan & s, const Params & p)
{
    throw NotImplementedException();

    std::pair<LaserScan, LaserScan> out = make_pair(LaserScan(), LaserScan());
    return out;
}
