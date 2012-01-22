/*
 * CircleIdentif.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CircleIdentif.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CircleIdentif::Params::Params()
{
}

std::string CircleIdentif::Params::getInfo()
{
    std::stringstream ss;
    ss << "CircleIdentif params :" << std::endl;
    //ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan CircleIdentif::apply(const LaserScan & raw, const Params & p)
{
    LaserScan out = raw;
    return out;
}
