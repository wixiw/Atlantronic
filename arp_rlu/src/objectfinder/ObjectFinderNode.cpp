/*
 * ObjectFinderNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "objectfinder/ObjectFinderNode.hpp"

using namespace arp_rlu;

ObjectFinderNode::ObjectFinderNode() :
    nh()
{

}

ObjectFinderNode::~ObjectFinderNode()
{
}

void ObjectFinderNode::go()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectFinderNode");
    ObjectFinderNode node;

    node.go();

    return 0;
}
