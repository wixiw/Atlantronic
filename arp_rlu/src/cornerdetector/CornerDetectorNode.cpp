/*
 * CornerDetectorNode.cpp
 *
 *  Created on: 28 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "CornerDetectorNode.hpp"

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
        scan(0, i) = fmod(s->angle_min + i * s->angle_increment + 2. * PI, 2. * PI);
        scan(1, i) = s->ranges[i];
    }

    //    cd.setScan(scan);
    //    cd.compute();
}

bool CornerDetectorNode::detectCornerCallback(DetectCorner::Request& req, DetectCorner::Response& res)
{
//    req.minAngle
//    req.maxAngle

    // Crop scan
//    unsigned int n = 0;
//    for(unsigned int i = 0; i < scan.cols() ; i++)
//    {
//        if( scan(0,i) < )
//        {
//
//        }
//    }

    cd.setScan(scan);
    Corner c = cd.compute();

    res.d1 = c.d1;
    res.alpha1 = c.alpha1;
    res.d2 = c.d2;
    res.alpha2 = c.alpha2;
    res.diag = c.diag;
    res.theta = c.theta;
    res.cornerAngleInDeg = c.cornerAngle * 180. / PI;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CornerDetector");
    CornerDetectorNode node;
    node.go();

    return 0;
}
