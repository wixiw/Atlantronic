/*
 * LaserOnlyLocalizator.cpp
 *
 *  Created on: 6 Mai 2012
 *      Author: Boris
 */

#include <KFL/LaserOnlyLocalizator.hpp>

#include "KFL/Logger.hpp"
#include "LSL/tools/JsonScanParser.hpp"

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
, dcp(lsl::DuoCircleIdentif::Params())
, minNbPoints(3)
, cartStddevMax(0.05)
, xMinAccessible(-1.4)
, xMaxAccessible(1.4)
, yMinAccessible(-0.9)
, yMaxAccessible(0.9)
, H_hky_robot(Pose2D())
, referencedBeacons()
{
    mfp.width = 0;

    pcp.minRange = 0.01;
    pcp.maxRange = 10.0;
    pcp.minTheta = -PI;
    pcp.maxTheta = PI;

    psp.rangeThres = 0.08;

    minNbPoints = 3;
    cartStddevMax = 0.03;

    xMinAccessible = -1.4;
    xMaxAccessible =  1.4;
    yMinAccessible = -0.9;
    yMaxAccessible =  0.9;

    cip.radius = 0.04;
    cip.coeffs = std::vector<double>();
    cip.coeffs.push_back( 1.0);
    cip.coeffs.push_back( 0.034);

    tcp.radiusTolerance = 0.03;
    tcp.distanceTolerance = 0.;
    tcp.maxLengthTolerance = 0.1;
    tcp.medLengthTolerance = 0.1;
    tcp.minLengthTolerance = 0.1;

    dcp.radiusTolerance = 0.03;
    dcp.distanceTolerance = 0.;
    dcp.lengthTolerance = 0.03;
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
    ss << cip.getInfo();
    ss << "****************************" << std::endl;
    ss << tcp.getInfo();
    ss << "****************************" << std::endl;
    ss << dcp.getInfo();
    ss << "****************************" << std::endl;
    ss << " [*] referencedBeacons : " << std::endl;
    for( unsigned int i = 0 ; i < referencedBeacons.size() ; i++ )
    {
        ss << "      [" << i << "] x: " << referencedBeacons[i].x() << "  - y: " << referencedBeacons[i].y() << "  - r: " << referencedBeacons[i].r() << std::endl;
    }
    ss << "****************************" << std::endl;
    ss << " [*] minNbPoints : " << minNbPoints << std::endl;
    ss << " [*] cartStddevMax : " << cartStddevMax << " (m)" << std::endl;
    ss << " [*] xMinAccessible : " << xMinAccessible << " (m)" << std::endl;
    ss << " [*] xMaxAccessible : " << xMaxAccessible << " (m)" << std::endl;
    ss << " [*] yMinAccessible : " << yMinAccessible << " (m)" << std::endl;
    ss << " [*] yMaxAccessible : " << yMaxAccessible << " (m)" << std::endl;
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
    if(!dcp.checkConsistency())
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "DuoCircleIdentif::Params is not consistent";
        return false;
    }
    if(xMinAccessible >= xMaxAccessible)
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "xMinAccessible >= xMaxAccessible";
        return false;
    }
    if(yMinAccessible >= yMaxAccessible)
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::Params::checkConsistency" << " - " << "yMinAccessible >= yMaxAccessible";
        return false;
    }
    return true;
}

LaserOnlyLocalizator::LaserOnlyLocalizator()
: params()
, lastEstimate(arp_math::Pose2D())
, lastRotation(0.)
, mfTimer()
, psTimer()
, fiTimer()
, ciTimer()
, tcTimer()
, dcTimer()
, globalTimer()
{

}


bool LaserOnlyLocalizator::process(lsl::LaserScan ls)
{
    globalTimer.Start();

    if(ls.getSize() == 0)
    {
        Log( NOTICE ) << "LaserOnlyLocalizator::process" << " - " << "Scan is empty => return false";
        globalTimer.Stop();
        return false;
    }


    //*****************************
    // Median filtering
    mfTimer.Start();
    LaserScan scan_0 = MedianFilter::apply(ls, params.mfp);
    mfTimer.Stop();


    //*****************************
    // Polar croping
    pcTimer.Start();
    LaserScan scan_1 = PolarCrop::apply(scan_0, params.pcp);
    pcTimer.Stop();
    clTimer.Start();
    scan_1.cleanUp();
    clTimer.Stop();
    cartTimer.Start();
    scan_1.computeCartesianData();
    cartTimer.Stop();


    //*****************************
    // Segmentation
    psTimer.Start();
    std::vector<DetectedObject> rawDObj = PolarSegment::apply( scan_1, params.psp);
    psTimer.Stop();


    //*****************************
    // Filtering on candidates : do not consider :
    // * too small DetectedObject
    // * DetectedObject touching borders of initial scan
    // * DetectedObject with big cartesian stddev
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
        if( rawDObj[i].getCartesianStddev().maxCoeff() > params.cartStddevMax )
        {
            continue;
        }
        detectedObjects.push_back( rawDObj[i] );
    }
    fiTimer.Stop();


    //*****************************
    // Interpretation as Circle
    ciTimer.Start();
    detectedCircles = CircleIdentif::apply(detectedObjects, params.cip);
    ciTimer.Stop();

    if(params.referencedBeacons.size() < 3)
    {
        Log( ERROR ) << "LaserOnlyLocalizator::process - Less than 3 beacons registered (actually "<< params.referencedBeacons.size() << ") => return false";
        globalTimer.Stop();
        return false;
    }


    //*****************************
    // Find Triangle
    tcTimer.Start();
    std::vector< std::vector<Circle> > vrc;
    vrc.push_back(params.referencedBeacons);
    std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > > triangle;
    triangle = TrioCircleIdentif::apply( detectedCircles, vrc, params.tcp );
    if(!triangle.empty())
    {
        Log( INFO ) << "LaserOnlyLocalizator::process - Triangle detection succeed";
        std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
        vpdc.push_back( std::make_pair( triangle[0].first[0], triangle[0].second[0] ));
        vpdc.push_back( std::make_pair( triangle[0].first[1], triangle[0].second[1] ));
        vpdc.push_back( std::make_pair( triangle[0].first[2], triangle[0].second[2] ));
        bool out = estimateFromConstellation( vpdc, lastEstimate );
        tcTimer.Stop();
        globalTimer.Stop();
        return out;
    }
    Log( INFO ) << "LaserOnlyLocalizator::process - Triangle detection failed";
    tcTimer.Stop();


    //*****************************
    // Find Segment
    dcTimer.Start();
    std::vector< std::pair<Circle,Circle> > refSegments;
    std::vector<Eigen::VectorXi> combs = combinaisons( params.referencedBeacons.size() , 2 );
    for(unsigned int i = 0 ; i < combs.size() ; i++)
    {
        refSegments.push_back( std::make_pair( params.referencedBeacons[combs[i](0)], params.referencedBeacons[combs[i](1)] ) );
    }
    std::vector< std::pair< std::pair<DetectedCircle, DetectedCircle>, std::pair<Circle,Circle> > > segments;
    segments = DuoCircleIdentif::apply( detectedCircles, refSegments, params.dcp );
    for(unsigned int i = 0 ; i < segments.size() ; i++)
    {
        lsl::Circle A,B;
        A = segments[i].second.first;
        B = segments[i].second.second;

        std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdcAB;
        vpdcAB.push_back( std::make_pair( segments[i].first.first,  segments[i].second.first  ));
        vpdcAB.push_back( std::make_pair( segments[i].first.second, segments[i].second.second ));
        arp_math::EstimatedPose2D estimAB;
        bool okAB = estimateFromConstellation( vpdcAB, estimAB );

        std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdcBA;
        vpdcBA.push_back( std::make_pair( segments[i].first.second, segments[i].second.first  ));
        vpdcBA.push_back( std::make_pair( segments[i].first.first,  segments[i].second.second ));
        arp_math::EstimatedPose2D estimBA;
        bool okBA = estimateFromConstellation( vpdcBA, estimBA );

        if( okAB && !okBA )
        {
            lastEstimate = estimAB;
            Log( INFO  ) << "LaserOnlyLocalizator::process - Segment detection succeed";
            dcTimer.Stop();
            globalTimer.Stop();
            return true;
        }
        if( !okAB && okBA )
        {
            lastEstimate = estimBA;
            Log( INFO ) << "LaserOnlyLocalizator::process - Segment detection succeed";
            dcTimer.Stop();
            globalTimer.Stop();
            return true;
        }
    }
    Log( INFO ) << "LaserOnlyLocalizator::process - Segment detection failed";
    dcTimer.Stop();

    globalTimer.Stop();
    return false;
}


arp_math::EstimatedPose2D LaserOnlyLocalizator::getEstimatedPose()
{
    return lastEstimate;
}


arp_math::Rotation2 LaserOnlyLocalizator::getRelativeHeadingForConfirmation()
{
    return arp_math::Rotation2(M_PI/3.);
}


bool LaserOnlyLocalizator::estimateFromConstellation(const std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > & vpdc, arp_math::EstimatedPose2D & pose )
{
    if( vpdc.size() < 2)
        return false;

    std::vector<Eigen::VectorXi> combs = combinaisons( vpdc.size(), 2 );

    Eigen::VectorXd angles = Eigen::VectorXd(combs.size());
    for(unsigned int i = 0 ; i < combs.size() ; i++)
    {
        Eigen::Vector3d aDC;
        Eigen::Vector3d bDC;
        aDC << vpdc[combs[i](0)].first.x(), vpdc[combs[i](0)].first.y(), 0.;
        bDC << vpdc[combs[i](1)].first.x(), vpdc[combs[i](1)].first.y(), 0.;

        Eigen::Vector3d aRC;
        Eigen::Vector3d bRC;
        aRC << vpdc[combs[i](0)].second.x(), vpdc[combs[i](0)].second.y(), 0.;
        bRC << vpdc[combs[i](1)].second.x(), vpdc[combs[i](1)].second.y(), 0.;

        Eigen::Vector3d Z(0.,0.,1.);
        Eigen::Vector3d nDC = (bDC - aDC).cross(Z);
        Eigen::Vector3d nRC = (bRC - aRC).cross(Z);
        nDC.normalize();
        nRC.normalize();

        angles(i) = atan2( nDC.cross(nRC)(2) , nDC.dot(nRC) );
    }
    double h = arp_math::mean(angles);

    Eigen::Vector3d baryDC = Eigen::Vector3d::Zero();
    Eigen::Vector3d baryRC = Eigen::Vector3d::Zero();
    for(unsigned int i = 0 ; i < vpdc.size() ; i++)
    {
        baryDC += Eigen::Vector3d(vpdc[i].first.x(), vpdc[i].first.y(), 0.);
        baryRC += Eigen::Vector3d(vpdc[i].second.x(), vpdc[i].second.y(), 0.);
    }
    baryDC /= vpdc.size();
    baryRC /= vpdc.size();

    double x = baryRC(0) - baryDC(0) * cos(h) + baryDC(1) * sin(h);
    double y = baryRC(1) - baryDC(0) * sin(h) - baryDC(1) * cos(h);

    pose.x( x );
    pose.y( y );
    pose.h( h );

    if( x < params.xMinAccessible || x > params.xMaxAccessible )
        return false;
    if( y < params.yMinAccessible || y > params.yMaxAccessible )
        return false;


    return true;
}

void LaserOnlyLocalizator::setParams(kfl::LaserOnlyLocalizator::Params p)
{
    params = p;
    params.tcp.distanceTolerance = 0.;
    params.dcp.distanceTolerance = 0.;
    return;
}

std::string LaserOnlyLocalizator::getPerformanceReport()
{
    std::stringstream info;
    info << "============================================" << std::endl;
    info << "LaserOnlyLocalizator Performance Report (ms)" << std::endl;
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
    return info.str();
}
