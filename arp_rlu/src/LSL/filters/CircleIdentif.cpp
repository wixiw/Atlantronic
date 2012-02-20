/*
 * CircleIdentif.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "CircleIdentif.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CircleIdentif::Params::Params()
: radius(0.04)
, rangeDelta(0.034)
{
}

std::string CircleIdentif::Params::getInfo()
{
    std::stringstream ss;
    ss << "CircleIdentif params :" << std::endl;
    ss << " [*] radius: " << radius << " (m)" << std::endl;
    ss << " [*] rangeDelta: " << rangeDelta << " (m)" << std::endl;
    return ss.str();
}

bool CircleIdentif::Params::checkConsistency()
{
    throw NotImplementedException();
    return false;
}

DetectedCircle CircleIdentif::apply(const DetectedObject & raw, const Params & p)
{
    throw NotImplementedException();

    DetectedCircle out;
    return out;
}

std::vector<DetectedCircle> CircleIdentif::apply(const std::vector<DetectedObject> & raws, const Params & p)
{
    throw NotImplementedException();

    std::vector<DetectedCircle> outs;
    return outs;
}
