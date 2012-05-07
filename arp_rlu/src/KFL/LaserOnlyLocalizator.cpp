/*
 * LaserOnlyLocalizator.cpp
 *
 *  Created on: 6 Mai 2012
 *      Author: Boris
 */

#include <KFL/LaserOnlyLocalizator.hpp>

#include "KFL/Logger.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace kfl;
using namespace lsl;
using namespace arp_core::log;

LaserOnlyLocalizator::Params::Params()
: mfp(lsl::MedianFilter::Params())
, pcp(lsl::PolarCrop::Params())
, psp(lsl::PolarSegment::Params())
, cip(lsl::CircleIdentif::Params())
, tcp(lsl::TrioCircleIdentif::Params())
, referencedBeacons()
, minNbPoints(3)
, H_hky_robot(Pose2D())
{
    mfp.width = 0;

    pcp.minRange = 0.01;
    pcp.maxRange = 10.0;
    pcp.minTheta = -PI;
    pcp.maxTheta = PI;

    psp.rangeThres = 0.08;

    minNbPoints = 3;

    cip.radius = 0.04;
    cip.coeffs = std::vector<double>();
    cip.coeffs.push_back( 1.0);
    cip.coeffs.push_back( 0.034);

    tcp.radiusTolerance = 0.03;
    tcp.distanceTolerance = 0.3;
    tcp.maxLengthTolerance = 0.1;
    tcp.medLengthTolerance = 0.1;
    tcp.minLengthTolerance = 0.1;
}

std::string LaserOnlyLocalizator::Params::getInfo() const
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << mfp.getInfo();
    ss << "****************************" << std::endl;
    ss << pcp.getInfo();
    ss << "****************************" << std::endl;
    ss << psp.getInfo();
    ss << "****************************" << std::endl;
    ss << "[*] minNbPoints : " << minNbPoints;
    ss << "****************************" << std::endl;
    ss << cip.getInfo();
    ss << "****************************" << std::endl;
    ss << tcp.getInfo();
    ss << "****************************" << std::endl;
    ss << " [*] referencedBeacons : " << std::endl;
    for( unsigned int i = 0 ; i < referencedBeacons.size() ; i++ )
    {
        ss << "      [" << i << "] x: " << referencedBeacons[i].x() << "  - y: " << referencedBeacons[i].y() << "  - r: " << referencedBeacons[i].r() << std::endl;
    }
    ss << "****************************" << std::endl;
    ss << " [*] minNbPoints : " << minNbPoints << std::endl;
    ss << " [*] H_hky_robot : " << H_hky_robot.toString() << std::endl;
    ss << std::endl;
    return ss.str();
}

bool LaserOnlyLocalizator::Params::checkConsistency() const
{
    if(!mfp.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "MedianFilter::Params is not consistent";
        return false;
    }
    if(!pcp.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "PolarCrop::Params is not consistent";
        return false;
    }
    if(!psp.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "PolarSegment::Params is not consistent";
        return false;
    }
    if(!cip.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "CircleIdentif::Params is not consistent";
        return false;
    }
    if(!tcp.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "TrioCircleIdentif::Params is not consistent";
        return false;
    }
    return true;
}


bool LaserOnlyLocalizator::process(lsl::LaserScan ls)
{
    return false;
}
