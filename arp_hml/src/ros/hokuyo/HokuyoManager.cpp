/*
 * HokuyoManager.cpp
 *
 *  Created on: 26 mai 2011
 *      Author: Boris
 */

#include "HokuyoManager.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

using namespace arp_hml;
using namespace std_msgs;

HokuyoManager::HokuyoManager(ros::NodeHandle &nh) :
    isRunning(true)
{
    emergency_sub = nh.subscribe("/Protokrot/emergency_stop", 1, &HokuyoManager::auCallback, this);
}

HokuyoManager::~HokuyoManager()
{
    ;
}

void HokuyoManager::auCallback(const std_msgs::BoolConstPtr& au)
{
    emergency = au->data;
}

void HokuyoManager::spin()
{
    ros::Rate r(10);
    while (ros::ok())
    {
        emergency = false;
        ros::spinOnce();
        r.sleep();

        if (emergency)
        {
            if (isRunning)
            {
                ROS_INFO("Emergency !! Shutdown Hokuyo node");
                system("rosnode kill hokuyo_node");
                isRunning = false;
            }
            break;
        }
    }
}

void HokuyoManager::shutDown()
{
}

HokuyoManager * node;

void quit(int sig)
{
    ROS_INFO("Ctrl C catched => Shuting down HokuyoManager");
    node->shutDown();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HokuyoManager");

    ros::NodeHandle nh("HokuyoManager");
    node = new HokuyoManager(nh);
    signal(SIGINT, quit);

    node->spin();
    ros::shutdown();
    exit(0);

    return true;
}

