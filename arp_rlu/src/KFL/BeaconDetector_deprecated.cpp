/*
 * BeaconDetector_deprecated.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BeaconDetector_deprecated.hpp"

#include "KFL/Logger.hpp"

#include "LSL/tools/JsonScanParser.hpp"
#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace kfl;
using namespace lsl;
using namespace arp_core::log;

BeaconDetector_deprecated::Params::Params()
: mfp(lsl::MedianFilter::Params())
, pcp(lsl::PolarCrop::Params())
, ccp(lsl::CartesianCrop::Params())
, psp(lsl::PolarSegment::Params())
, cip(lsl::CircleIdentif::Params())
, maxDistance2NearestReferencedBeacon(0.7)
{
    mfp.width = 0;

    pcp.minRange = 0.01 * VectorXd::Ones(1);
    pcp.maxRange = 10.0 * VectorXd::Ones(1);
    pcp.minTheta = -PI;
    pcp.maxTheta = PI;

    ccp.minX = -1.5;
    ccp.maxX =  1.5;
    ccp.minY = -1.0;
    ccp.maxY =  1.0;

    psp.rangeThres = 0.08;

    cip.radius = 0.04;
    cip.rangeDelta = 0.034;
}

std::string BeaconDetector_deprecated::Params::getInfo() const
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << mfp.getInfo();
    ss << "****************************" << std::endl;
    ss << pcp.getInfo();
    ss << "****************************" << std::endl;
    ss << ccp.getInfo();
    ss << "****************************" << std::endl;
    ss << psp.getInfo();
    ss << "****************************" << std::endl;
    ss << cip.getInfo();
    ss << std::endl;
    return ss.str();
}

bool BeaconDetector_deprecated::Params::checkConsistency() const
{
    if(!mfp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "MedianFilter::Params is not consistent";
        return false;
    }
    if(!pcp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "PolarCrop::Params is not consistent";
        return false;
    }
    if(!ccp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "CartesianCrop::Params is not consistent";
        return false;
    }
    if(!psp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "PolarSegment::Params is not consistent";
        return false;
    }
    if(!cip.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "CircleIdentif::Params is not consistent";
        return false;
    }
    if( maxDistance2NearestReferencedBeacon < 0.  )
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::Params::checkConsistency" << " - " << "maxDistance2NearestReferencedBeacon < 0.";
        return false;
    }
    return true;
}

BeaconDetector_deprecated::BeaconDetector_deprecated()
: params(Params())
, detectedCircles(std::vector< lsl::DetectedCircle >())
, referencedBeaconUsed(std::vector<bool>())
, referencedBeacons(std::vector< lsl::Circle >())
, deltaTime(0.)
{
}

BeaconDetector_deprecated::~BeaconDetector_deprecated()
{
}

bool BeaconDetector_deprecated::process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh)
{
    if(ls.getSize() == 0)
    {
        Log( NOTICE ) << "BeaconDetector_deprecated::process" << " - " << "Scan is empty => return false";
        return false;
    }

    deltaTime = (ls.getPolarData().block(0,1,1,ls.getSize()-1) - ls.getPolarData().block(0,0,1,ls.getSize()-1)).minCoeff();

    LaserScan scan_0 = MedianFilter::apply(ls, params.mfp);
//    export_json( scan_0, "./BeaconDetector_deprecated__process__scan_0.json" );

    LaserScan scan_1 = PolarCrop::apply(scan_0, params.pcp);
    scan_1.cleanUp();
//    export_json( scan_1, "./BeaconDetector_deprecated__process__scan_1.json" );

    scan_1.computeCartesianData(tt, xx, yy, hh);

    LaserScan scan_2 = CartesianCrop::apply(scan_1, params.ccp);
//    export_json( scan_2, "./BeaconDetector_deprecated__process__scan_2.json" );

    std::vector<DetectedObject> v = PolarSegment::apply( scan_2, params.psp);
//    for(unsigned int i = 0 ; i < v.size() ; i++)
//    {
//        std::stringstream ss;
//        ss << "./BeaconDetector_deprecated__process__detectedobject_" << i << ".json";
//                export_json( v[i].getScan(), ss.str() );
//    }

    detectedCircles = CircleIdentif::apply(v, params.cip);
    referencedBeaconUsed = std::vector<bool>();
    for(unsigned int i = 0 ; i < referencedBeacons.size() ; i++)
    {
        referencedBeaconUsed.push_back(false);
    }
    return true;
}

void BeaconDetector_deprecated::setReferencedBeacons(std::vector<lsl::Circle> beacons)
{
    referencedBeacons = beacons;
    return;
}

bool BeaconDetector_deprecated::getBeacon(double t, lsl::Circle & target, Eigen::Vector2d & meas)
{
    if(referencedBeacons.size() == 0)
    {
        Log( ERROR ) << "BeaconDetector_deprecated::getBeacon" << " - " << "No beacon registered => return false";
        return false;
    }
    if(detectedCircles.size() < 2)
    {
        return false;
    }
    for(unsigned int i = 0 ; i < detectedCircles.size() ; i++)
    {
        if( t <= detectedCircles[i].getApparentCenterTime() + deltaTime/2.
                && detectedCircles[i].getApparentCenterTime() - deltaTime/2. < t)
        {
            double x = detectedCircles[i].x();
            double y = detectedCircles[i].y();
            Eigen::VectorXd dist = 10. * Eigen::VectorXd::Ones(referencedBeacons.size());
            for(unsigned int j = 0 ; j < referencedBeacons.size() ; j++)
            {
                if(!referencedBeaconUsed[j])
                {
                    dist(j) = sqrt( (x-referencedBeacons[j].x())*(x-referencedBeacons[j].x())
                            + (y-referencedBeacons[j].y())*(y-referencedBeacons[j].y()) );
                }
            }

            int jMinDist = -1;
            double minDist = dist.minCoeff(&jMinDist);

            if( minDist < params.maxDistance2NearestReferencedBeacon )
            {
                referencedBeaconUsed[jMinDist] = true;
                meas(0) = detectedCircles[i].getApparentCenterRange();
                meas(1) = detectedCircles[i].getApparentCenterTheta();
                target = referencedBeacons[jMinDist];
                return true;
            }
            else
            {
                Log(DEBUG) << "BeaconDetector_deprecated::getBeacon -    target is too far ("<< minDist << " > " << params.maxDistance2NearestReferencedBeacon << ")";
            }
        }
    }
    return false;
}

std::vector< lsl::DetectedCircle > BeaconDetector_deprecated::getDetectedCircles()
{
    return detectedCircles;
}

void BeaconDetector_deprecated::setParams(kfl::BeaconDetector_deprecated::Params p)
{
    params = p;
    return;
}
