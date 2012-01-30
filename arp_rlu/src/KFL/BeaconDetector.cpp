/*
 * BeaconDetector.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BeaconDetector.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace kfl;

BeaconDetector::Params::Params()
{
}

std::string BeaconDetector::Params::getInfo()
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << mfp.getInfo();
    ss << "****************************" << std::endl;
    ss << pcp.getInfo();
    ss << "****************************" << std::endl;
    ss << ccp.getInfo();
    ss << "****************************" << std::endl;
    ss << psp.getInfo();
    ss << std::endl;
    return ss.str();
}

BeaconDetector::BeaconDetector()
{
    throw NotImplementedException();
}

BeaconDetector::~BeaconDetector()
{
}

bool BeaconDetector::process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh)
{
    throw NotImplementedException();
    return false;
}

void BeaconDetector::setRefecencedBeacons(std::vector<lsl::Circle> beacons)
{
    throw NotImplementedException();
    return;
}

bool BeaconDetector::getBeacon(double t, std::pair<lsl::Circle, Eigen::Vector3d > & p)
{
    throw NotImplementedException();
    return false;
}

void BeaconDetector::setParams(kfl::BeaconDetector::Params)
{
    throw NotImplementedException();
    return;
}
