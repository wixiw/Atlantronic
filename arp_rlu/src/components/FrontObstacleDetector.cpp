/*
 * FrontObstacleDetector.cpp
 *
 *  Created on: 14 mai 2012
 *      Author: ard
 */

#include <iomanip>

#include "FrontObstacleDetector.hpp"
#include <rtt/Component.hpp>
#include "LSL/Logger.hpp"
#include "LSL/tools/JsonScanParser.hpp"
#include <ros/ros.h>


using namespace arp_core::log;
using namespace arp_rlu;
using namespace lsl;
using namespace arp_math;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::FrontObstacleDetector )

FrontObstacleDetector::FrontObstacleDetector(const std::string& name)
: RluTaskContext(name)
, mfp(lsl::MedianFilter::Params())
, pcp(lsl::PolarCrop::Params())
, psp(lsl::PolarSegment::Params())
, minNbPoints(3)
, cartStddevMax(0.05)
, cip(lsl::CircleIdentif::Params())
, xMinAccessible(-1.4)
, xMaxAccessible(1.4)
, yMinAccessible(-0.9)
, yMaxAccessible(0.9)
, H_hky_robot(Pose2D())
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::lsl::Logger::InitFile("LSL", DEBUG);

    createOrocosInterface();
    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getTime();

    mfp.width = 3;

    pcp.minRange = 0.05;
    pcp.maxRange = 3.2;
    pcp.minTheta = -PI/2.;
    pcp.maxTheta = PI/2.;

    psp.rangeThres = 0.08;

    minNbPoints = 3;
    cartStddevMax = 0.03;

    cip.radius = 0.04;
    cip.coeffs = std::vector<double>();
    cip.coeffs.push_back( 1.0);
    cip.coeffs.push_back( 0.034);

    xMinAccessible = -1.4;
    xMaxAccessible =  1.4;
    yMinAccessible = -0.9;
    yMaxAccessible =  0.9;

    H_hky_robot = arp_math::Pose2D(0.01, 0., 0.);


}

bool FrontObstacleDetector::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void FrontObstacleDetector::updateHook()
{
    RluTaskContext::updateHook();

    //**************************************************************
    // Acquisition
    EstimatedPose2D H_robot_table;
    if( RTT::NoData == inPose.read(H_robot_table) )
    {
        return;
    }
    EstimatedPose2D H_hky_table = H_robot_table * H_hky_robot;
    sensor_msgs::LaserScan rosScan;
    if( RTT::NoData == inScan.read(rosScan) )
    {
        return;
    }
    double dateBeg = rosScan.header.stamp.toSec() - m_monotonicTimeToRealTime;
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
    export_json( scan_0, "./FrontObstacleDetector__process__scan_0.json" );


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
    export_json( scan_1, "./FrontObstacleDetector__process__scan_1.json" );

}


std::string FrontObstacleDetector::coGetPerformanceReport()
{
    return "";
}


void FrontObstacleDetector::createOrocosInterface()
{
    addPort("inScan",inScan)
                    .doc("LaserScan from LRF");

    addPort("inPose",inPose)
    .doc("H_robot_table");

    addPort("outFrontObstacles",outFrontObstacles)
    .doc("List of things detected with front hokuyo");
}
