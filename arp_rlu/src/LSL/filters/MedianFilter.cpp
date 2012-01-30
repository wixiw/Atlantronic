/*
 * MedianFilter.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "MedianFilter.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

MedianFilter::Params::Params()
: width(3)
{
}

std::string MedianFilter::Params::getInfo()
{
    std::stringstream ss;
    ss << "MedianFilter params :" << std::endl;
    ss << " [*] width: " << width << std::endl;
    return ss.str();
}

LaserScan MedianFilter::apply(const LaserScan & raw, const Params & p)
{
    throw NotImplementedException();

    LaserScan out = raw;
    return out;
}
