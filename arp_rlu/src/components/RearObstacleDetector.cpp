/*
 * RearObstacleDetector.cpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#include <iomanip>

#include "RearObstacleDetector.hpp"
#include <rtt/Component.hpp>
#include "LSL/Logger.hpp"
#include "LSL/tools/JsonScanParser.hpp"
#include <ros/ros.h>
#include "ObstacleParams.hpp"

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_time;
using namespace lsl;
using namespace arp_math;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::RearObstacleDetector )

RearObstacleDetector::RearObstacleDetector(const std::string& name)
: RluTaskContext(name)
, mfp(lsl::MedianFilter::Params())
, pcp(lsl::PolarCrop::Params())
, psp(lsl::PolarSegment::Params())
, minNbPoints(3)
, cartStddevMax(0.20)
, cip(lsl::CircleIdentif::Params())
, H_hky_robot(Pose2D())
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::lsl::Logger::InitFile("LSL", WARN);

    createOrocosInterface();
    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getAbsoluteTime();

    mfp.width = 3;

    pcp.minRange = 0.05;
    pcp.maxRange = 3.2;
    pcp.minTheta = -PI/2.;
    pcp.maxTheta = PI/2.;

    psp.rangeThres = 0.08;

    minNbPoints = 3;
    cartStddevMax = 0.15;
    opponentRadius = 0.15;

    cip.radius = opponentRadius;
    cip.coeffs = std::vector<double>();
    cip.coeffs.push_back( 1.0);
    cip.coeffs.push_back( opponentRadius );

    H_hky_robot = arp_math::Pose2D(-0.154, 0.0, M_PI);


}

bool RearObstacleDetector::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void RearObstacleDetector::updateHook()
{
    //LOG( Info ) << "Update" << endlog();
    //**************************************************************
    // Acquisition
    EstimatedPose2D H_robot_table;
    if( RTT::NoData == inPose.read(H_robot_table) )
    {
        //LOG( Info ) << "RearObstacleDetector - No data on inPose port" << endlog();
        return;
    }
    EstimatedPose2D H_hky_table = H_robot_table * H_hky_robot;
    sensor_msgs::LaserScan rosScan;
    if( RTT::NoData == inScan.read(rosScan) )
    {
        //LOG( Info ) << "RearObstacleDetector - No data on inScan port" << endlog();
        return;
    }
    ArdTimeDelta dateBeg = rosScan.header.stamp.toSec() - m_monotonicTimeToRealTime;
    LaserScan lslScan;
    Eigen::MatrixXd polarData(3, rosScan.ranges.size());
    for (unsigned int i = 0; i != rosScan.ranges.size(); i++)
    {
        polarData(0,i) = dateBeg + i * rosScan.time_increment;
        polarData(2,i) = - rosScan.angle_min - i*rosScan.angle_increment;
        if (rosScan.ranges[i] <= rosScan.range_max && rosScan.range_min <= rosScan.ranges[i])
        {
            polarData(1,i) = rosScan.ranges[i];
        }
        else
        {
            polarData(1,i) = 0.;
        }
    }
    lslScan.setPolarData(polarData);


    //*****************************
    // Median filtering
    mfTimer.Start();
    LaserScan scan_0 = MedianFilter::apply(lslScan, mfp);
    mfTimer.Stop();
        //export_json( scan_0, "./RearObstacleDetector__process__scan_0.json" );

    //*****************************
    // Polar croping
    pcTimer.Start();
    LaserScan scan_1 = PolarCrop::apply(scan_0, pcp);
    pcTimer.Stop();
    clTimer.Start();
    scan_1.cleanUp();
    clTimer.Stop();
    cartTimer.Start();
    Eigen::VectorXd ttc = scan_1.getTimeData();
    Eigen::VectorXd xxc = Eigen::VectorXd::Ones(scan_1.getSize()) * H_hky_table.x();
    Eigen::VectorXd yyc = Eigen::VectorXd::Ones(scan_1.getSize()) * H_hky_table.y();
    Eigen::VectorXd hhc = Eigen::VectorXd::Ones(scan_1.getSize()) * H_hky_table.h();
    scan_1.computeCartesianData(ttc, xxc, yyc, hhc);
    cartTimer.Stop();
        //export_json( scan_1, "./RearObstacleDetector__process__scan_1.json" );


    //*****************************
    // Segmentation
    psTimer.Start();
    std::vector<DetectedObject> rawDObj = PolarSegment::apply( scan_1, psp);
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
        if(rawDObj[i].getScan().getSize() < minNbPoints)
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
        if( rawDObj[i].getCartesianStddev().maxCoeff() > cartStddevMax )
        {
            continue;
        }
        detectedObjects.push_back( rawDObj[i] );
    }
    fiTimer.Stop();


    //*****************************
    // Interpretation as Circle
    ciTimer.Start();
    detectedCircles = CircleIdentif::apply(detectedObjects, cip);
    ciTimer.Stop();


    //*****************************
    // Détection des obstacles
    obsTimer.Start();
    detectedObstacles.clear();
    for(unsigned int i = 0 ; i < detectedCircles.size() ; i++)
    {
        if( (detectedCircles[i].getCartesianMean()[0] < OBSTACLE_X_MIN) || (detectedCircles[i].getCartesianMean()[0] > OBSTACLE_X_MAX) )
        {
            continue;
        }
        if( (detectedCircles[i].getCartesianMean()[1] < OBSTACLE_Y_MIN) || (detectedCircles[i].getCartesianMean()[1] > OBSTACLE_Y_MAX) )
        {
            continue;
        }
        detectedObstacles.push_back( arp_math::Vector2( detectedCircles[i].x(), detectedCircles[i].y() ) );
    }
    obsTimer.Stop();

    outObstacles.write(detectedObstacles);
}


std::string RearObstacleDetector::coGetPerformanceReport()
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
    info << "  [#] Obstacle Extraction duration   : mean=" << obsTimer.GetMeanElapsedTime()*1000.;
    info << "  , stddev=" << sqrt(obsTimer.GetStdDevElapsedTime())*1000.;
    info << "  , min=" << obsTimer.GetMinElapsedTime()*1000.;
    info << "  , max=" << obsTimer.GetMaxElapsedTime()*1000.<< std::endl;
    return info.str();
}


void RearObstacleDetector::createOrocosInterface()
{
    addPort("inScan",inScan)
                                            .doc("LaserScan from LRF");

    addPort("inPose",inPose)
    .doc("H_robot_table");

    addPort("outObstacles",outObstacles)
    .doc("List of things detected with front hokuyo");

    addProperty("MedianFilterWidth", mfp.width);

    addProperty("PolarCropMinRange", pcp.minRange);
    addProperty("PolarCropMaxRange", pcp.maxRange);
    addProperty("PolarCropMinTheta", pcp.minTheta);
    addProperty("PolarCropMaxTheta", pcp.maxTheta);

    addProperty("PolarSegmentRangeThreshold", psp.rangeThres);

    addProperty("MinNbPoints", minNbPoints);
    addProperty("cartStddevMax", cartStddevMax);
    addProperty("opponentRadius", opponentRadius);
}
