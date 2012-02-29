/*
 * CircleIdentif.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */


#include "CircleIdentif.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

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
    {
        Log( NOTICE ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radius <= 0.)";
        return false;
    }
    if( rangeDelta > radius )
    {
        Log( NOTICE ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (rangeDelta > radius)";
        return false;
    }
    return true;
}

DetectedCircle CircleIdentif::apply(const DetectedObject & raw, const Params & p)
{
    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "CircleIdentif::apply" << " - " << "Parameters are not consistent => Return DetectedCircle()";
        return DetectedCircle();
    }

    DetectedCircle out(raw);
    out.r(p.radius);

    Vector2 pov = raw.getApparentPointOfView();
    double aov = raw.getApparentAngleOfView();
    double acmr = raw.getApparentCartesianMeanRange();
    double acmt = raw.getApparentCartesianMeanTheta();

    out.x( pov(0) + (acmr + p.rangeDelta) * cos(aov + acmt) );
    out.y( pov(1) + (acmr + p.rangeDelta) * sin(aov + acmt) );

    out.setApparentCenterRange(acmr + p.rangeDelta);
    out.setApparentCenterTheta(acmt);
    out.setApparentCenterTime( raw.getApparentCartesianMeanTime() );

    return out;
}

std::vector<DetectedCircle> CircleIdentif::apply(const std::vector<DetectedObject> & raws, const Params & p)
{
    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "CircleIdentif::apply (vector version)" << " - " << "Parameters are not consistent => Return empty std::vector<DetectedCircle>";
        return std::vector<DetectedCircle>();
    }
    std::vector<DetectedCircle> outs;
    for(int i = 0 ; i < raws.size() ; i++)
    {
        outs.push_back( CircleIdentif::apply(raws[i], p) );
    }
    return outs;
}
