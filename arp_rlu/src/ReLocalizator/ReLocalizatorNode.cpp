/*
 * ReLocalizatorNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "ReLocalizatorNode.hpp"

using namespace arp_rlu;


ReLocalizatorNode::ReLocalizatorNode() :
    nh()
{
    estimatePosition_srv = nh.advertiseService("/ReLocalizator/EstimatePosition", &ReLocalizatorNode::estimatePositionCallback, this);
    cornerdetection_client_ = nh.serviceClient<arp_rlu::DetectCorner>("/CornerDetector/DetectCorner");
}
ReLocalizatorNode::~ReLocalizatorNode()
{
}

bool ReLocalizatorNode::estimatePositionCallback(EstimatePosition::Request& req, EstimatePosition::Response& res)
{
    ROS_INFO("***************************");
    ROS_INFO("Estimation asked");
    ROS_INFO("================");


    rl.previousX = req.previousX;
    rl.previousY = req.previousY;
    rl.previousTheta = req.previousTheta;

    TableCorner target = rl.selectTargetTableCorner();
    ROS_INFO("TableCorner targeted :");
    target.print();

    std::pair<double, double> p = rl.chooseScanWindow(target);
    ROS_INFO("Selected window in Hokuyo ref : from %f to %f", arp_math::rad2deg(p.first), arp_math::rad2deg(p.second));

    arp_rlu::DetectCorner dc_srv;
    dc_srv.request.minAngle = p.first;
    dc_srv.request.maxAngle = p.second;
    if ( !cornerdetection_client_.call(dc_srv))
    {
        res.estimatedX = rl.previousX;
        res.estimatedY = rl.previousY;
        res.estimatedTheta = rl.previousTheta;
        res.quality = -1;
        ROS_WARN("EstimatePosition Service : calling CornerDetection service failed");
        return true;
    }

    Corner detected;
    detected.d1 = dc_srv.response.d1;
    detected.alpha1 = dc_srv.response.alpha1;
    detected.length1 = dc_srv.response.length1;
    detected.angleBegin1 = dc_srv.response.angleBegin1;
    detected.angleEnd1 = dc_srv.response.angleEnd1;
    detected.d2 = dc_srv.response.d2;
    detected.alpha2 = dc_srv.response.alpha2;
    detected.length2 = dc_srv.response.length2;
    detected.angleBegin2 = dc_srv.response.angleBegin2;
    detected.angleEnd2 = dc_srv.response.angleEnd2;
    detected.diag = dc_srv.response.diag;
    detected.theta = dc_srv.response.theta;
    detected.cornerAngle = dc_srv.response.cornerAngleInDeg;

    rl.estimatePose(detected, target);

    res.estimatedX = rl.estimatedX;
    res.estimatedY = rl.estimatedY;
    res.estimatedTheta = rl.estimatedTheta;
    res.quality = rl.quality;

    if( rl.quality > 0 )
    {
        ROS_INFO("SUCESS !!");
    }
    else
    {
        ROS_WARN("FAILED !!");
    }

    return true;
}

void ReLocalizatorNode::go()
{
    ros::spin();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ReLocalizator");
    ReLocalizatorNode node;

    node.go();

    return 0;
}
