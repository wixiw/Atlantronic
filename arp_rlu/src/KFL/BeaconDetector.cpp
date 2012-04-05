/*
 * BeaconDetector.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BeaconDetector.hpp"

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

BeaconDetector::Params::Params()
: mfp(lsl::MedianFilter::Params())
, pcp(lsl::PolarCrop::Params())
, psp(lsl::PolarSegment::Params())
, cip(lsl::CircleIdentif::Params())
, tcp(lsl::TrioCircleIdentif::Params())
, dcp(lsl::DuoCircleIdentif::Params())
, minNbPoints(4)
{
    mfp.width = 0;

    pcp.minRange = 0.01 * VectorXd::Ones(1);
    pcp.maxRange = 10.0 * VectorXd::Ones(1);
    pcp.minTheta = -PI;
    pcp.maxTheta = PI;

    psp.rangeThres = 0.08;

    minNbPoints = 4;

    cip.radius = 0.04;
    cip.rangeDelta = 0.034;

    tcp.radiusTolerance = 0.03;
    tcp.distanceTolerance = 0.3;
    tcp.maxLengthTolerance = 0.1;
    tcp.medLengthTolerance = 0.1;
    tcp.minLengthTolerance = 0.1;

    dcp.radiusTolerance = 0.03;
    dcp.distanceTolerance = 0.3;
    dcp.lengthTolerance = 0.05;
}

std::string BeaconDetector::Params::getInfo() const
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << mfp.getInfo();
    ss << "****************************" << std::endl;
    ss << pcp.getInfo();
    ss << "****************************" << std::endl;
    ss << psp.getInfo();
    ss << "****************************" << std::endl;
    ss << cip.getInfo();
    ss << std::endl;
    return ss.str();
}

bool BeaconDetector::Params::checkConsistency() const
{
    if(!mfp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "MedianFilter::Params is not consistent";
        return false;
    }
    if(!pcp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "PolarCrop::Params is not consistent";
        return false;
    }
    if(!psp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "PolarSegment::Params is not consistent";
        return false;
    }
    if(!cip.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "CircleIdentif::Params is not consistent";
        return false;
    }
    if(!tcp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "TrioCircleIdentif::Params is not consistent";
        return false;
    }
    if(!dcp.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "DuoCircleIdentif::Params is not consistent";
        return false;
    }
    return true;
}

BeaconDetector::BeaconDetector()
: params(Params())
, detectedCircles(std::vector< lsl::DetectedCircle >())
, referencedBeacons(std::vector< lsl::Circle >())
, deltaTime(0.)
{
}

BeaconDetector::~BeaconDetector()
{
}

bool BeaconDetector::process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh)
{
    // Clear last result
    foundBeacons.clear();

    if(ls.getSize() == 0)
    {
        Log( NOTICE ) << "BeaconDetector::process" << " - " << "Scan is empty => return false";
        return false;
    }

    deltaTime = (ls.getPolarData().block(0,1,1,ls.getSize()-1) - ls.getPolarData().block(0,0,1,ls.getSize()-1)).minCoeff();

    //*****************************
    // Median filtering
    LaserScan scan_0 = MedianFilter::apply(ls, params.mfp);
//    export_json( scan_0, "./BeaconDetector__process__scan_0.json" );

    //*****************************
    // Polar croping
    LaserScan scan_1 = PolarCrop::apply(scan_0, params.pcp);
    scan_1.cleanUp();
    scan_1.computeCartesianData(tt, xx, yy, hh);
//    export_json( scan_1, "./BeaconDetector__process__scan_1.json" );


    //*****************************
    // Segmentation
    std::vector<DetectedObject> rawDObj = PolarSegment::apply( scan_1, params.psp);

//    for(unsigned int i = 0 ; i < rawDObj.size() ; i++)
//    {
//        std::stringstream ss;
//        ss << "./BeaconDetector__process__raw_detectedobject_" << i << ".json";
//        export_json( rawDObj[i].getScan(), ss.str() );
//    }

    //*****************************
    // Filtering on candidates : do not considere too small DetectedObject and DetectedObject touching borders of initial scan
    std::vector<DetectedObject> dObj;
    for(unsigned int i = 0 ; i < rawDObj.size() ; i++)
    {
        if(rawDObj[i].getScan().getSize() < params.minNbPoints)
        {
            continue;
        }
        if( rawDObj[i].getScan().getPolarData()(2,0) == scan_0.getPolarData()(2,0) )
        {
            continue;
        }
        if( rawDObj[i].getScan().getPolarData()(2,rawDObj[i].getScan().getSize()-1) == scan_0.getPolarData()(2,scan_0.getSize()-1) )
        {
            continue;
        }
        dObj.push_back( rawDObj[i] );

    }

//    for(unsigned int i = 0 ; i < dObj.size() ; i++)
//    {
//        std::stringstream ss;
//        ss << "./BeaconDetector__process__filtered_detectedobject_" << i << ".json";
//        export_json( dObj[i].getScan(), ss.str() );
//    }


    //*****************************
    // Interpretation as Circle
    detectedCircles = CircleIdentif::apply(dObj, params.cip);

    if(referencedBeacons.size() < 3)
    {
        Log( ERROR ) << "BeaconDetector::process - Less than 3 beacons registered (actually "<< referencedBeacons.size() << ") => return false";
        return false;
    }

    Log( DEBUG ) << "BeaconDetector::process - " << detectedCircles.size() << " circles detected";

    //*****************************
    // Find Triangle
    std::vector< std::vector<Circle> > vrc;
    vrc.push_back(referencedBeacons);
    std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > > triangle;
    triangle = TrioCircleIdentif::apply( detectedCircles, vrc, params.tcp );
    if(!triangle.empty())
    {
        Eigen::Vector3d angles;
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            angles(i) = betweenMinusPiAndPlusPi(triangle[0].first[i].getApparentCenterTheta());
        }
        int iFirst, iSecond, iThird;
        angles.minCoeff(&iFirst);
        angles.maxCoeff(&iThird);
        iSecond = 3 - iFirst - iThird;
        foundBeacons.push_back( std::make_pair( triangle[0].first[iFirst],  triangle[0].second[iFirst] ) );
        foundBeacons.push_back( std::make_pair( triangle[0].first[iSecond], triangle[0].second[iSecond] ) );
        foundBeacons.push_back( std::make_pair( triangle[0].first[iThird],  triangle[0].second[iThird] ) );
        Log( DEBUG ) << "BeaconDetector::process - Triangle detected";
        return true;
    }
    Log( DEBUG ) << "BeaconDetector::process - Triangle detection failed => try segment detection...";

    //*****************************
    // Find Segment
    std::vector< std::pair<Circle,Circle> > refSegments;
    refSegments.push_back( std::make_pair( referencedBeacons[1], referencedBeacons[2] ) );
    refSegments.push_back( std::make_pair( referencedBeacons[0], referencedBeacons[2] ) );
    refSegments.push_back( std::make_pair( referencedBeacons[0], referencedBeacons[1] ) );
    std::vector< std::pair< std::pair<DetectedCircle, DetectedCircle>, std::pair<Circle,Circle> > > segments;
    segments = DuoCircleIdentif::apply( detectedCircles, refSegments, params.dcp );
    if(!segments.empty())
    {
        Eigen::Vector2d angles;
        angles(0) = betweenMinusPiAndPlusPi(segments[0].first.first.getApparentCenterTheta());
        angles(1) = betweenMinusPiAndPlusPi(segments[0].first.second.getApparentCenterTheta());
        if( angles(0) < angles(1) )
        {
            foundBeacons.push_back( std::make_pair(segments[0].first.first, segments[0].second.first ) );
            foundBeacons.push_back( std::make_pair(segments[0].first.second, segments[0].second.second ) );
        }
        else
        {
            foundBeacons.push_back( std::make_pair(segments[0].first.second, segments[0].second.second ) );
            foundBeacons.push_back( std::make_pair(segments[0].first.first, segments[0].second.first ) );
        }
        Log( DEBUG ) << "BeaconDetector::process - Segment [ (" << foundBeacons[0].second.x() << "," << foundBeacons[0].second.y() << ") ; (" << foundBeacons[1].second.x() << "," << foundBeacons[1].second.y() << ") ] detected";
        return true;
    }
    Log( DEBUG ) << "BeaconDetector::process - Neither triangle nor segment has been detected :-(";
    return false;
}

void BeaconDetector::setReferencedBeacons(std::vector<lsl::Circle> beacons)
{
    referencedBeacons = beacons;
    return;
}

bool BeaconDetector::getBeacon(double t, lsl::Circle & target, Eigen::Vector2d & meas)
{
    for(unsigned int i = 0 ; i < foundBeacons.size() ; i++)
    {
        if( t <= foundBeacons[i].first.getApparentCenterTime() + deltaTime/2.
                && foundBeacons[i].first.getApparentCenterTime() - deltaTime/2. < t)
        {
            meas(0) = foundBeacons[i].first.getApparentCenterRange();
            meas(1) = foundBeacons[i].first.getApparentCenterTheta();
            target = foundBeacons[i].second.getPosition();
            return true;
        }
    }
    return false;
}

std::vector< lsl::DetectedCircle > BeaconDetector::getDetectedCircles()
{
    return detectedCircles;
}


std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > BeaconDetector::getFoundBeacons()
{
    return foundBeacons;
}

void BeaconDetector::setParams(kfl::BeaconDetector::Params p)
{
    params = p;
    return;
}
