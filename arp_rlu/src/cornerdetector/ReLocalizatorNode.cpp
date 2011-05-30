/*
 * ReLocalizatorNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "ReLocalizatorNode.hpp"

using namespace arp_rlu;

TableCorner::TableCorner() :
        x( 0.0 ),
        y( 0.0 ),
        type( NONE )
{

}

bool TableCorner::isVisibleFrom(double x, double y, double minAngle, double maxAngle)
{
    return true;
}

ReLocalizatorNode::ReLocalizatorNode() :
    nh(),
    estimatedX(0.0),
    estimatedY(0.0),
    estimatedTheta(0.0),
    quality(-1)
{
    estimatePosition_srv = nh.advertiseService("EstimatePosition", &ReLocalizatorNode::estimatePositionCallback, this);
}
ReLocalizatorNode::~ReLocalizatorNode()
{
}

bool ReLocalizatorNode::estimatePositionCallback(EstimatePosition::Request& req, EstimatePosition::Response& res)
{
    previousX = req.previousX;
    previousY = req.previousY;
    previousTheta = req.previousTheta;


    res.estimatedX = estimatedX;
    res.estimatedY = estimatedY;
    res.estimatedTheta = estimatedTheta;
    res.quality = quality;
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
    for (unsigned int i = 0; i < tableCorners.size(); i++)
    {
        std::cout << "TableCorner " << i << std::endl;
        std::cout << "  x : " << tableCorners[i].x << std::endl;
        std::cout << "  y : " << tableCorners[i].y << std::endl;

        switch(tableCorners[i].type)
        {
            case NORTH_WEST:
                std::cout << "  type : NORTH_WEST" << std::endl;
                break;
            case NORTH_EAST:
                std::cout << "  type : NORTH_EAST" << std::endl;
                break;
            case SOUTH_EAST:
                std::cout << "  type : SOUTH_EAST" << std::endl;
                break;
            case SOUTH_WEST:
                std::cout << "  type : SOUTH_WEST" << std::endl;
                break;
            default :
                std::cout << "  type : NONE" << std::endl;
                break;
        }

    }
}

TableCorner ReLocalizatorNode::selectTargetTableCorner()
{
    return TableCorner();
}

std::pair<double, double> ReLocalizatorNode::chooseScanWindow(TableCorner target)
{
    return std::make_pair(0.0,0.0);
}

void ReLocalizatorNode::estimatePose(Corner detected, TableCorner target)
{
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ReLocalizator");
    ReLocalizatorNode node;

    std::vector<TableCorner> vtc;
    node.setTableCorners(vtc);
    node.printTableCorners();

    node.go();

    return 0;
}
