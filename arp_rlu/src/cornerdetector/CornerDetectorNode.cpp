/*
 * CornerDetectorNode.cpp
 *
 *  Created on: 28 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "CornerDetector.hpp"

using namespace arp_rlu;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CornerDetector");

    CornerDetector cd;
    ros::spin();

    return 0;
}
