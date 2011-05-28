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

    scan_sub = nh.subscribe("/scan", 1, &HokuyoManager::scanCallback, this);

    last_run_time_ = ros::WallTime::now();
    last_kill_time_ = ros::WallTime::now();

}

HokuyoManager::~HokuyoManager()
{
    ;
}

void HokuyoManager::auCallback(const std_msgs::BoolConstPtr& au)
{
    emergency = au->data;
}

void HokuyoManager::scanCallback(LaserScanConstPtr scan)
{
    scanReceived = true;
}

void HokuyoManager::spin()
{
    ros::Rate r(10);
    while (ros::ok())
    {
        scanReceived = false;
        emergency = false;
        ros::spinOnce();
        r.sleep();

        if (emergency)
        {
            if (isRunning)
            {
                ROS_INFO("Emergency !! Shutdown Hokuyo node");
                system("rosnode kill hokuyo_node");
                last_kill_time_ = ros::WallTime::now();
                isRunning = false;
            }
            break;
        }
//        else
//        {
//            if (isRunning)
//            {
//                if (!scanReceived && (ros::WallTime::now() - last_run_time_ > ros::WallDuration(2.0)))
//                {
//                    ROS_INFO("No scan found... Hokuyo node seems to be sick. Shut it down");
//                    system("rosnode kill hokuyo_node");
//                    last_kill_time_ = ros::WallTime::now();
//                    isRunning = false;
//                }
//                continue;
//            }
//            else
//            {
//                if ((ros::WallTime::now() - last_kill_time_ > ros::WallDuration(5.0)))
//                {
//                    ROS_INFO("Running Hokuyo node...");
//                    system("rosrun hokuyo_node hokuyo_node &");
//                    last_run_time_ = ros::WallTime::now();
//                    isRunning = true;
//                }
//                continue;
//            }
//        }
    }
}

void HokuyoManager::shutDown()
{
//    if (isRunning)
//    {
//        ROS_INFO("Shutdown Hokuyo node");
//        system("rosnode kill hokuyo_node");
//    }

}

HokuyoManager * m;

void quit(int sig)
{
    ROS_INFO("Ctrl C catched => Shuting down HokuyoManager");
    m->shutDown();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HokuyoManager");

    ros::NodeHandle nh("HokuyoManager");
    m = new HokuyoManager(nh);
    signal(SIGINT, quit);

    m->spin();
    ros::shutdown();
    exit(0);

    return true;
}

