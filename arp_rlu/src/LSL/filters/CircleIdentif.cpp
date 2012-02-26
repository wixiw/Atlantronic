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

bool CircleIdentif::Params::checkConsistency() const
{
    if(radius <= 0.)
        return false;
    if( rangeDelta > radius )
        return false;
    return true;
}

DetectedCircle CircleIdentif::apply(const DetectedObject & raw, const Params & p)
{
    DetectedCircle out(raw);


    out.r(p.radius);



    return out;
}

std::vector<DetectedCircle> CircleIdentif::apply(const std::vector<DetectedObject> & raws, const Params & p)
{

    std::vector<DetectedCircle> outs;
    for(int i = 0 ; i < raws.size() ; i++)
    {
        outs.push_back( CircleIdentif::apply(raws[i], p) );
    }
    return outs;
}
