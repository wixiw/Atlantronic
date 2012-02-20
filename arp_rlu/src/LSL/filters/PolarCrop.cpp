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
: ParamsInterface()
, minRange(0.1 * VectorXd::Ones(1))
, maxRange(10.0 * VectorXd::Ones(1))
, minTheta(-PI)
, maxTheta(PI)
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
    ss << " [*] minTheta : " << rad2deg(minTheta) << " (deg)"<< std::endl;
    ss << " [*] maxTheta : " << rad2deg(maxTheta) << " (deg)"<< std::endl;
    return ss.str();
}

bool PolarCrop::Params::checkConsistency()
{
    throw NotImplementedException();
    return false;
}

LaserScan PolarCrop::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    LaserScan out = raw;
    return out;
}
