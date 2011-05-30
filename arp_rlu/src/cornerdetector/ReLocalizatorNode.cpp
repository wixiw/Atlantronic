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
    double x = req.previousX;
    double y = req.previousY;
    double theta = req.previousTheta;

    res.estimatedX = 0.0;
    res.estimatedY = 0.0;
    res.estimatedTheta = 0.0;
    res.quality = -1.;
    return true;
}

void ReLocalizatorNode::go()
{
    ros::spin();
}


void ReLocalizatorNode::setTableCorners(std::vector<TableCorner> tc)
{
    tableCorners = tc;
}

void ReLocalizatorNode::printTableCorners()
{
    std::cout << "*****************************" << std::endl;
    std::cout << "Nb of TableCorners :" << tableCorners.size() << std::endl;
    for(unsigned int i = 0; i < tableCorners.size() ; i++)
    {
        std::cout << "TableCorner " << i << std::endl;
        std::cout << "  x : " << tableCorners[i].x << std::endl;
        std::cout << "  y : " << tableCorners[i].y << std::endl;
        std::cout << "  type : " << tableCorners[i].type << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ReLocalizator");
    ReLocalizatorNode node;

    std::vector<TableCorner> vtc;
    node.setTableCorners( vtc );


    node.go();

    return 0;
}
