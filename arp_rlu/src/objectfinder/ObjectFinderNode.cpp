/*
 * ObjectFinderNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "objectfinder/ObjectFinderNode.hpp"

using namespace arp_rlu;
using namespace sensor_msgs;
using namespace arp_math;

ObjectFinderNode::ObjectFinderNode() :
    nh(), scan(Eigen::MatrixXd::Zero(1, 1)), reverse_scan(false)
{
    std::string front_scan_topic_name;
    if (nh.getParam("ObjectFinder/scan_topic_name", front_scan_topic_name))
    {
        ROS_INFO("Got param named 'scan_topic_name' : %s", front_scan_topic_name);
    }
    else
    {
        ROS_WARN("Failed to get param 'scan_topic_name'. Take default value (/top_scan)");
        front_scan_topic_name = "/top_scan";
    }

    if (nh.getParam("ObjectFinder/reverse_scan", reverse_scan))
    {
        ROS_INFO("Got param named 'reverse_scan' : %i", reverse_scan);
    }
    else
    {
        ROS_WARN("Failed to get param 'reverse_scan'. Take default value (false)");
        reverse_scan = false;
    }

    scan_sub = nh.subscribe(front_scan_topic_name, 1, &ObjectFinderNode::scanCallback, this);
    findobjects_srv = nh.advertiseService("/ObjectFinder/FindObjects", &ObjectFinderNode::findobjectsCallback,
            this);
}

ObjectFinderNode::~ObjectFinderNode()
{
}

void ObjectFinderNode::go()
{
    ros::spin();
}

void ObjectFinderNode::scanCallback(LaserScanConstPtr s)
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

bool ObjectFinderNode::findobjectsCallback(FindObjects::Request& req, FindObjects::Response& res)
{
    if (scan.rows() < 2)
    {
        ROS_WARN("ObjectFinderNode findobjectsCallback : scan is empty");
        return false;
    }

    ROS_INFO("ObjectFinderNode findobjectsCallback");

    objf.setPolarScan(scan);
    objf.computeCartesianScan(req.xRobot, req.yRobot, req.thetaRobot);
    objf.onTableOnly();
    std::vector<Scan> vect = objf.clusterize();

//    cd.setScan(cropScan(req.minAngle, req.maxAngle));
//
//    Corner c = cd.compute();
//
//    res.d1 = c.d1;
//    res.alpha1 = c.alpha1;
//    res.length1 = c.length1;
//    res.angleBegin1 = c.angleBegin1;
//    res.angleEnd1 = c.angleEnd1;
//    res.d2 = c.d2;
//    res.alpha2 = c.alpha2;
//    res.length2 = c.length2;
//    res.angleBegin2 = c.angleBegin2;
//    res.angleEnd2 = c.angleEnd2;
//    res.diag = c.diag;
//    res.theta = c.theta;
//    res.cornerAngleInDeg = c.cornerAngle * 180. / PI;

    for(unsigned int i = 0; i < vect.size() ; i++)
    {
        ROS_INFO("Cluster %d - size: %d", i, vect[i].cols());
    }

    res.confidence = vect.size();
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectFinderNode");
    ObjectFinderNode node;

    node.go();

    return 0;
}
