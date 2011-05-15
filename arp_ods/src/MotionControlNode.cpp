#include <ros/ros.h>
#include "MotionControl.hpp"

using namespace arp_ods;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MotionControl");

    MotionControl mc(ros::this_node::getName());
    ros::spin();

    return 0;
}

