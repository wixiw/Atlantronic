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
    estimatePosition_srv = nh.advertiseService("EstimatePosition", &ReLocalizatorNode::estimatePositionCallback, this);
}
ReLocalizatorNode::~ReLocalizatorNode()
{
}

bool ReLocalizatorNode::estimatePositionCallback(EstimatePosition::Request& req, EstimatePosition::Response& res)
{
    ROS_INFO("bite");
    res.x = 0.0;
    res.y = 0.0;
    res.theta = 0.0;
    res.quality = -1.;
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
