/*
 * CornerDetectorNode.cpp
 *
 *  Created on: 28 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "CornerDetectorNode.hpp"
#include "math/math.hpp"

using namespace arp_rlu;
using namespace arp_math;
using namespace sensor_msgs;

CornerDetectorNode::CornerDetectorNode() :
    nh()
{
    scan_sub = nh.subscribe("/scan", 1, &CornerDetectorNode::scanCallback, this);
    detectcorner_srv = nh.advertiseService("DetectCorner", &CornerDetectorNode::detectCornerCallback, this);
}
CornerDetectorNode::~CornerDetectorNode()
{
}

void CornerDetectorNode::go()
{
    ros::spin();
}

void CornerDetectorNode::scanCallback(LaserScanConstPtr s)
{
    unsigned int n = s->ranges.size();
    scan = Eigen::MatrixXd(2, n);
    for (unsigned int i = 0; i != s->ranges.size(); i++)
    {
        scan(0, i) = s->angle_min + i * s->angle_increment;
        scan(1, i) = s->ranges[i];
    }

    //    cd.setScan(scan);
    //    cd.compute();
}

bool CornerDetectorNode::detectCornerCallback(DetectCorner::Request& req, DetectCorner::Response& res)
{

    cd.setScan(cropScan(req.minAngle, req.maxAngle));

    Corner c = cd.compute();

    res.d1 = c.d1;
    res.alpha1 = c.alpha1;
    res.length1 = c.length1;
    res.angleBegin1 = c.angleBegin1;
    res.angleEnd1 = c.angleEnd1;
    res.d2 = c.d2;
    res.alpha2 = c.alpha2;
    res.length2 = c.length2;
    res.angleBegin2 = c.angleBegin2;
    res.angleEnd2 = c.angleEnd2;
    res.diag = c.diag;
    res.theta = c.theta;
    res.cornerAngleInDeg = c.cornerAngle * 180. / PI;
    return true;
}

Scan CornerDetectorNode::cropScan(double minAngle, double maxAngle)
{

    std::cout << "Scan minAngle :" << scan.row(0).minCoeff() << std::endl;
    std::cout << "Scan maxAngle :" << scan.row(0).maxCoeff() << std::endl;
    std::cout << "asked minAngle :" << minAngle << std::endl;
    std::cout << "asked maxAngle :" << maxAngle << std::endl;

    // Crop scan
    unsigned int n = 0;
    for (unsigned int i = 0; i < scan.cols(); i++)
    {
        if (betweenMinusPiAndPlusPi(scan(0, i)) > betweenMinusPiAndPlusPi(minAngle) && betweenMinusPiAndPlusPi(
                scan(0, i)) < betweenMinusPiAndPlusPi(maxAngle))
        {
            n++;
        }
    }
    if(n == 0)
    {
        return Eigen::MatrixXd::Zero(2,2);
    }
    Scan cropedScan(2, n);
    unsigned int index = 0;
    for (unsigned int i = 0; i < scan.cols(); i++)
    {
        if (betweenMinusPiAndPlusPi(scan(0, i)) > betweenMinusPiAndPlusPi(minAngle) && betweenMinusPiAndPlusPi(
                scan(0, i)) < betweenMinusPiAndPlusPi(maxAngle))
        {
            cropedScan(0, index) = scan(0, i);
            cropedScan(1, index) = scan(1, i);
            index++;
        }
    }

    std::cout << "Croped Scan minAngle :" << cropedScan.row(0).minCoeff() << std::endl;
    std::cout << "Croped Scan maxAngle :" << cropedScan.row(0).maxCoeff() << std::endl;

    return cropedScan;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CornerDetector");
    CornerDetectorNode node;
    node.go();

    return 0;
}
