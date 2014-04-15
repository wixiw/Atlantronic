/*
 * ActuatorGraphicsNode.cpp
 *
 *  Created on: 11 mars 2014
 *      Author: wla
 */
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <iostream>
#include "ActuatorsFrameGlade_2014.hpp"

arp_hml::ActuatorsFrameGlade_2014 * node;

void quit(int sig)
{
    ROS_INFO("Ctrl C catched => Shuting down HokuyoManager");
    node->shutDown();
    delete(node);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ActuatorGraphics");
    ros::NodeHandle nh("ActuatorGraphics");
    node = new arp_hml::ActuatorsFrameGlade_2014();
    if( NULL == node)
    {
        ROS_ERROR("No more memory");
        return false;
    }

    signal(SIGINT, quit);

    if( false == node->init(argc, argv) )
    {
        ROS_ERROR("Failed to init GraphicNode");
        delete(node);
        return -1;
    }

    ros::Rate r(10);

    while (ros::ok() && node->spin())
    {
        r.sleep();
        ros::spinOnce();
    }

    delete(node);
    return 0;
}

