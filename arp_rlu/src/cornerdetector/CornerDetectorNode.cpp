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
    nh(), scan(Eigen::MatrixXd::Zero(1, 1)), reverse_scan(false)
{
    std::string front_scan_topic_name;
    if (nh.getParam("CornerDetector/front_scan_topic_name", front_scan_topic_name))
    {
        ROS_INFO("Got param named 'front_scan_topic_name' : %s", front_scan_topic_name);
    }
    else
    {
        ROS_WARN("Failed to get param 'front_scan_topic_name'. Take default value (/front_scan)");
        front_scan_topic_name = "/front_scan";
    }


    if (nh.getParam("CornerDetector/reverse_scan", reverse_scan))
    {
        ROS_INFO("Got param named 'reverse_scan' : %i", reverse_scan);
    }
    else
    {
        ROS_WARN("Failed to get param 'reverse_scan'. Take default value (true)");
        reverse_scan = true;
    }

    scan_sub = nh.subscribe(front_scan_topic_name, 1, &CornerDetectorNode::scanCallback, this);
    detectcorner_srv = nh.advertiseService("/CornerDetector/DetectCorner", &CornerDetectorNode::detectCornerCallback,
            this);
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
    if (!reverse_scan)
    {
        for (unsigned int i = 0; i != n; i++)
        {
            scan(0, i) = s->angle_min + i * s->angle_increment;
            scan(1, i) = s->ranges[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i != n; i++)
        {
            scan(0, n-1-i) = betweenMinusPiAndPlusPi( - s->angle_min - i * s->angle_increment );
            scan(1, n-1-i) = s->ranges[i];
        }
    }

}

bool CornerDetectorNode::detectCornerCallback(DetectCorner::Request& req, DetectCorner::Response& res)
{
    if (scan.rows() < 2)
    {
        ROS_WARN("CornerDetectorNode detectCornerCallback : scan is empty");
        return false;
    }

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

    if (scan.rows() < 2)
    {
        ROS_WARN("CornerDetectorNode cropScan : Scan is empty before croping");
        return Eigen::MatrixXd::Zero(1, 1);
    }

    ROS_INFO("Scan minAngle : %f", scan.row(0).minCoeff());
    ROS_INFO("Scan maxAngle : %f", scan.row(0).maxCoeff());
    ROS_INFO("asked minAngle : %f",minAngle);
    ROS_INFO("asked maxAngle : %f", maxAngle);

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
    if (n == 0)
    {
        ROS_WARN("CornerDetectorNode cropScan : Scan is empty after croping");
        return Eigen::MatrixXd::Zero(1, 1);
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

    return cropedScan;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CornerDetector");
    CornerDetectorNode node;
    node.go();

    return 0;
}
