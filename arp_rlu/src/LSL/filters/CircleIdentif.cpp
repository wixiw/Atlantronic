/*
 * CircleIdentif.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */


#include "CircleIdentif.hpp"

#include "LSL/Logger.hpp"
#include <math.h>

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

CircleIdentif::Params::Params()
: radius(0.04)
, coeffs(std::vector<double>())
{
    coeffs.push_back(1.);
    coeffs.push_back(0.034);
}

std::string CircleIdentif::Params::getInfo() const
{
    std::stringstream ss;
    ss << "CircleIdentif params :" << std::endl;
    ss << " [*] radius: " << radius << " (m)" << std::endl;
    ss << " [*] coeffs: ";
    for(unsigned int i = 0 ; i < coeffs.size() ; i++)
    {
        ss << coeffs[i] << " ; ";
    }
    ss << "" << std::endl;
    return ss.str();
}

bool CircleIdentif::Params::checkConsistency() const
{
    if(radius <= 0.)
    {
        Log( WARN ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radius <= 0.)";
        return false;
    }
    if( coeffs.empty() )
    {
        Log( WARN ) << "CircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (coeffs is empty)";
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

    double range = 0;
    for(unsigned int i = 0 ; i < p.coeffs.size() ; i++)
    {
        range += p.coeffs[i] * std::pow(acmr, p.coeffs.size()-i-1);
//        Log( DEBUG ) << "[i = " << i << "]" << "p.coeffs[i] = " << p.coeffs[i] << "  p.coeffs.size()-i-1 = " << p.coeffs.size()-i-1;
//        Log( DEBUG ) << "[i = " << i << "]" << "std::pow(acmr, p.coeffs.size()-i-1) = " << std::pow(acmr, p.coeffs.size()-i-1);
//        Log( DEBUG ) << "[i = " << i << "]" << "p.coeffs[i] * std::pow(acmr, p.coeffs.size()-i-1) = " << p.coeffs[i] * std::pow(acmr, p.coeffs.size()-i-1);
//        Log( DEBUG ) << "[i = " << i << "]" << "range = " << range;
    }

    out.x( pov(0) + range * cos(aov + acmt) );
    out.y( pov(1) + range * sin(aov + acmt) );

//    out.setApparentCenterRange(acmr + p.rangeDelta);
    out.setApparentCenterRange( range );
    out.setApparentCenterTheta(acmt);
    out.setApparentCenterTime( raw.getApparentCartesianMeanTime() );


//    Log( DEBUG ) << "*************************";
//    Log( DEBUG ) << p.getInfo();
//    Log( DEBUG ) << "pov = " << pov.transpose();
//    Log( DEBUG ) << "cartMean = " << cartMean.transpose();
//    Log( DEBUG ) << "aov = " << aov;
//    Log( DEBUG ) << "acmr = " << acmr;
//    Log( DEBUG ) << "acmt = " << acmr;
//    Log( DEBUG ) << "***";
//    Log( DEBUG ) << "range = " << range;
//    Log( DEBUG ) << "out.x() = " << out.x();
//    Log( DEBUG ) << "out.y() = " << out.y();
//    Log( DEBUG ) << "out.getApparentCenterRange() = " << out.getApparentCenterRange();
//    Log( DEBUG ) << "out.getApparentCenterTheta() = " << out.getApparentCenterTheta();
//    Log( DEBUG ) << "out.getApparentCenterTime() = " << out.getApparentCenterTime();

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
