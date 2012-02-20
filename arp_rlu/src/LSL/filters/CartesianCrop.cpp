/*
 * CartesianCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CartesianCrop.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

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

bool CartesianCrop::Params::checkConsistency()
{
    throw NotImplementedException();
    return false;
}

LaserScan CartesianCrop::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    LaserScan out = raw;
    return out;
}
