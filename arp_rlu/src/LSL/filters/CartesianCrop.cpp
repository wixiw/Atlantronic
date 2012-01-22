/*
 * CartesianCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CartesianCrop.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CartesianCrop::Params::Params()
{
}

std::string CartesianCrop::Params::getInfo()
{
    std::stringstream ss;
    ss << "CartesianCrop params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan CartesianCrop::apply(const LaserScan & raw, const Params & p)
{
    LaserScan out = raw;
    return out;
}
