/*
 * PolarCrop.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "PolarCrop.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

PolarCrop::Params::Params()
{
}

std::string PolarCrop::Params::getInfo()
{
    std::stringstream ss;
    ss << "PolarCrop params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan PolarCrop::apply(const LaserScan & raw, const Params & p)
{
    LaserScan out = raw;
    return out;
}
