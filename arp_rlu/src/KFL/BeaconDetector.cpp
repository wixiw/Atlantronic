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
, minNbPoints(3)
{
    mfp.width = 0;

    pcp.minRange = 0.01;
    pcp.maxRange = 10.0;
    pcp.minTheta = -PI;
    pcp.maxTheta = PI;

    psp.rangeThres = 0.08;

    minNbPoints = 3;

    xMin = -2.0;
    xMax =  2.0;
    yMin = -1.5;
    yMax =  1.5;

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
    ss << "[*] minNbPoints : " << minNbPoints;
    ss << "****************************" << std::endl;
    ss << cip.getInfo();
    ss << "****************************" << std::endl;
    ss << tcp.getInfo();
    ss << "****************************" << std::endl;
    ss << dcp.getInfo();
    ss << "****************************" << std::endl;
    ss << " [*] minNbPoints : " << minNbPoints << std::endl;
    ss << " [*] xMin        : " << xMin << " (m)" << std::endl;
    ss << " [*] xMax        : " << xMax << " (m)" << std::endl;
    ss << " [*] yMin        : " << yMin << " (m)" << std::endl;
    ss << " [*] yMax        : " << yMax << " (m)" << std::endl;
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
    globalTimer.Start();

    // Clear last result
    foundBeacons.clear();

    if(ls.getSize() == 0)
    {
        Log( NOTICE ) << "BeaconDetector::process" << " - " << "Scan is empty => return false";
        globalTimer.Stop();
        return false;
    }

    deltaTime = (ls.getPolarData().block(0,1,1,ls.getSize()-1) - ls.getPolarData().block(0,0,1,ls.getSize()-1)).minCoeff();

    //*****************************
    // Median filtering
    mfTimer.Start();
    LaserScan scan_0 = MedianFilter::apply(ls, params.mfp);
    mfTimer.Stop();
//    export_json( scan_0, "./BeaconDetector__process__scan_0.json" );

    //*****************************
    // Polar croping
    pcTimer.Start();
    LaserScan scan_1 = PolarCrop::apply(scan_0, params.pcp);
    pcTimer.Stop();
    clTimer.Start();
    scan_1.cleanUp();
    clTimer.Stop();
    cartTimer.Start();
    scan_1.computeCartesianData(tt, xx, yy, hh);
    cartTimer.Stop();
//    pcTimer.Stop();
//    export_json( scan_1, "./BeaconDetector__process__scan_1.json" );


    //*****************************
    // Segmentation
    psTimer.Start();
    std::vector<DetectedObject> rawDObj = PolarSegment::apply( scan_1, params.psp);
    psTimer.Stop();

//    for(unsigned int i = 0 ; i < rawDObj.size() ; i++)
//    {
//        std::stringstream ss;
//        ss << "./BeaconDetector__process__raw_detectedobject_" << i << ".json";
//        export_json( rawDObj[i].getScan(), ss.str() );
//    }

    //*****************************
    // Filtering on candidates : do not consider :
    // * too small DetectedObject
    // * DetectedObject touching borders of initial scan
    // * DetectedObject out of enlarged table
    fiTimer.Start();
    detectedObjects.clear();
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
        if( (rawDObj[i].getCartesianMean()[0] < params.xMin) || (rawDObj[i].getCartesianMean()[0] > params.xMax) )
        {
            continue;
        }
        if( (rawDObj[i].getCartesianMean()[1] < params.yMin) || (rawDObj[i].getCartesianMean()[1] > params.yMax) )
        {
            continue;
        }
        detectedObjects.push_back( rawDObj[i] );
    }
    fiTimer.Stop();

//    for(unsigned int i = 0 ; i < dObj.size() ; i++)
//    {
//        std::stringstream ss;
//        ss << "./BeaconDetector__process__filtered_detectedobject_" << i << ".json";
//        export_json( dObj[i].getScan(), ss.str() );
//    }


    //*****************************
    // Interpretation as Circle
    ciTimer.Start();
    detectedCircles = CircleIdentif::apply(detectedObjects, params.cip);
    ciTimer.Stop();

    if(referencedBeacons.size() < 3)
    {
        Log( ERROR ) << "BeaconDetector::process - Less than 3 beacons registered (actually "<< referencedBeacons.size() << ") => return false";
        globalTimer.Stop();
        return false;
    }

    Log( DEBUG ) << "BeaconDetector::process - " << detectedCircles.size() << " circles detected : ";
    for(int i = 0 ; i < detectedCircles.size() ; i++)
    {
        Log( DEBUG ) << "BeaconDetector::process -      [" << i << "] x:" << detectedCircles[i].x() << " - y:" << detectedCircles[i].y() << " - r:" << detectedCircles[i].r();
    }

    //*****************************
    // Find Triangle
    tcTimer.Start();
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
        tcTimer.Stop();
        globalTimer.Stop();
        return true;
    }
    Log( DEBUG ) << "BeaconDetector::process - Triangle detection failed => try segment detection...";
    tcTimer.Stop();

    //*****************************
    // Find Segment
    dcTimer.Start();
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
        dcTimer.Stop();
        globalTimer.Stop();
        return true;
    }
    Log( DEBUG ) << "BeaconDetector::process - Neither triangle nor segment has been detected :-(";

    dcTimer.Stop();
    globalTimer.Stop();
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

std::vector< lsl::DetectedObject > BeaconDetector::getDetectedObjects()
{
    return detectedObjects;
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


std::string BeaconDetector::getPerformanceReport()
{
    std::stringstream info;
    info << "============================================" << std::endl;
    info << "BeaconDetector Performance Report (ms)" << std::endl;
    info << "--------------------------------------------" << std::endl;
    info << "  [*] Number of samples used : " << globalTimer.GetRawRefreshTime().size() << std::endl;
    info << "  [*] Period                 : mean=" << globalTimer.GetMeanRefreshTime()*1000.;
    info << "  , stddev=" << sqrt(globalTimer.GetStdDevRefreshTime())*1000.;
    info << "  , min=" << globalTimer.GetMinRefreshTime()*1000.;
    info << "  , max=" << globalTimer.GetMaxRefreshTime()*1000. << std::endl;
    info << "  [*] Global Duration        : mean=" << globalTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(globalTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << globalTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << globalTimer.GetMaxElapsedTime()*1000. << std::endl;
    info << "--------------------------------------------" << std::endl;
    info << "  [#] MedianFilter duration    : mean=" << mfTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(mfTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << mfTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << mfTimer.GetMaxElapsedTime()*1000. << std::endl;
    info << "  [#] PolarCrop duration       : mean=" << pcTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(pcTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << pcTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << pcTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] CleanUp duration         : mean=" << clTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(clTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << clTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << clTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] Cartesian Compute duration : mean=" << cartTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(cartTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << cartTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << cartTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] PolarSegment duration    : mean=" << psTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(psTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << psTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << psTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] Filtering duration       : mean=" << fiTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(fiTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << fiTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << fiTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] CircleIdentif duration   : mean=" << ciTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(ciTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << ciTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << ciTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] Triangle Detect duration : mean=" << tcTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(tcTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << tcTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << tcTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "  [#] Segment Detect duration : mean=" << dcTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(dcTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << dcTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << dcTimer.GetMaxElapsedTime()*1000.<< std::endl;
    info << "============================================" << std::endl;
    return info.str();
}
