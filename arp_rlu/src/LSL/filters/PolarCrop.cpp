/*
 * PolarCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "PolarCrop.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

PolarCrop::Params::Params()
: minRange()
, maxRange()
, minTheta()
, maxTheta()
{
}

std::string PolarCrop::Params::getInfo()
{
    std::stringstream ss;
    ss << "PolarCrop params :" << std::endl;
    ss << " [*] minRange size     : " << minRange.size() << std::endl;
    ss << " [*] minRange minCoeff : " << minRange.minCoeff() << " (m)"<< std::endl;
    ss << " [*] minRange maxCoeff : " << minRange.maxCoeff() << " (m)"<< std::endl;
    ss << " [*] maxRange size     : " << maxRange.size() << std::endl;
    ss << " [*] maxRange minCoeff : " << maxRange.minCoeff() << " (m)"<< std::endl;
    ss << " [*] maxRange maxCoeff : " << maxRange.maxCoeff() << " (m)"<< std::endl;
    ss << " [*] minTheta size     : " << minTheta.size() << std::endl;
    ss << " [*] minTheta minCoeff : " << rad2deg(minTheta.minCoeff()) << " (deg)"<< std::endl;
    ss << " [*] minTheta maxCoeff : " << rad2deg(minTheta.maxCoeff()) << " (deg)"<< std::endl;
    ss << " [*] maxTheta size     : " << maxTheta.size() << std::endl;
    ss << " [*] maxTheta minCoeff : " << rad2deg(maxTheta.minCoeff()) << " (deg)"<< std::endl;
    ss << " [*] maxTheta maxCoeff : " << rad2deg(maxTheta.maxCoeff()) << " (deg)"<< std::endl;
    return ss.str();
}

LaserScan PolarCrop::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    LaserScan out = raw;
    return out;
}
