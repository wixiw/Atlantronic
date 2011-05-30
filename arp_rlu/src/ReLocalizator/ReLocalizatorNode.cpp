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
    rl.previousX = req.previousX;
    rl.previousY = req.previousY;
    rl.previousTheta = req.previousTheta;


    res.estimatedX = rl.estimatedX;
    res.estimatedY = rl.estimatedY;
    res.estimatedTheta = rl.estimatedTheta;
    res.quality = rl.quality;
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
