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

std::string CircleIdentif::Params::getInfo() const
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
        Log( WARN ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radius <= 0.)";
        return false;
    }
    if( rangeDelta > radius )
    {
        Log( WARN ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (rangeDelta > radius)";
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

    Vector2 pov = raw.getPointOfView();
    Vector2 cartMean = raw.getCartesianMean();
    double aov = raw.getAngleOfView();
    double acmr = raw.getApparentCartesianMeanRange();
    double acmt = raw.getApparentCartesianMeanTheta();

    out.x( cartMean(0) + p.rangeDelta * cos(aov + acmt) );
    out.y( cartMean(1) + p.rangeDelta * sin(aov + acmt) );

//    out.setApparentCenterRange(acmr + p.rangeDelta);
    out.setApparentCenterRange( arp_math::mean(raw.getScan().getPolarData().row(1)) + p.rangeDelta );
    out.setApparentCenterTheta(acmt);
    out.setApparentCenterTime( raw.getApparentCartesianMeanTime() );

    Log( DEBUG ) << "CircleIdentif::apply - [xMean] cartMean(0)=" << cartMean(0);
    Log( DEBUG ) << "CircleIdentif::apply - [yMean] cartMean(1)=" << cartMean(1);
    Log( DEBUG ) << "CircleIdentif::apply - [x] pov(0)=" << pov(0);
    Log( DEBUG ) << "CircleIdentif::apply - [y] pov(1)=" << pov(1);
    Log( DEBUG ) << "CircleIdentif::apply - [xMean - x] cartMean(0) - pov(0)=" << cartMean(0) - pov(0);
    Log( DEBUG ) << "CircleIdentif::apply - [yMean - y] cartMean(1) - pov(1)=" << cartMean(1) - pov(1);
    Log( DEBUG ) << "CircleIdentif::apply - [acmr] acmr=" << cartMean(1) - pov(1);
    Log( DEBUG ) << "CircleIdentif::apply - [atan2] aov + acmt=" << aov + acmt;
    Log( DEBUG ) << "CircleIdentif::apply - [xDelta] p.rangeDelta * cos(aov + acmt)=" << p.rangeDelta * cos(aov + acmt);
    Log( DEBUG ) << "CircleIdentif::apply - [yDelta] p.rangeDelta * sin(aov + acmt)=" << p.rangeDelta * sin(aov + acmt);
    Log( DEBUG ) << "CircleIdentif::apply - [xCenter] out.x()=" << out.x();
    Log( DEBUG ) << "CircleIdentif::apply - [yCenter] out.y()=" << out.y();
    Log( DEBUG ) << "CircleIdentif::apply - [ranges] ranges =" << raw.getScan().getPolarData().row(1);
    Log( DEBUG ) << "CircleIdentif::apply - [mean(range)] mean(range) =" << arp_math::mean(raw.getScan().getPolarData().row(1));
    Log( DEBUG ) << "CircleIdentif::apply - [range] mean(range) + rangeDelta=" << out.getApparentCenterRange();
    Log( DEBUG ) << "CircleIdentif::apply - [theta] acmt=" << out.getApparentCenterTheta();
    Log( DEBUG ) << "CircleIdentif::apply - [time] time=" << out.getApparentCenterTime();
    Log( DEBUG ) << "CircleIdentif::apply - --------------";

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
    for(unsigned int i = 0 ; i < raws.size() ; i++)
    {
        outs.push_back( CircleIdentif::apply(raws[i], p) );
    }
    return outs;
}
