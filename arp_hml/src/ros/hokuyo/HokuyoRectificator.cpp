/*
 * HokuyoRectificator.cpp
 *
 *  Created on: 2 juin 2011
 *      Author: boris
 */

#include "HokuyoRectificator.hpp"

using namespace arp_hml;

HokuyoRectificator::HokuyoRectificator() :
    nh(),
    bias(0.0503)
{
    scan_sub = nh.subscribe("/scan", 1, &HokuyoRectificator::scanCallback, this);
    rectifiedscan_pub = nh.advertise<sensor_msgs::LaserScan> ("/rectifiedscan", 100);
}

HokuyoRectificator::~HokuyoRectificator()
{
}

void HokuyoRectificator::scanCallback(LaserScanConstPtr s)
{
    sensor_msgs::LaserScan rectifiedscan;

    rectifiedscan.header.stamp = s->header.stamp;
    rectifiedscan.header.frame_id = s->header.frame_id;
    rectifiedscan.angle_min = s->angle_min;
    rectifiedscan.angle_max = s->angle_max;
    rectifiedscan.angle_increment = s->angle_increment;
    rectifiedscan.scan_time = s->scan_time;
    rectifiedscan.time_increment = s->time_increment;
    rectifiedscan.range_min = s->range_min + bias;
    rectifiedscan.range_max = s->range_max + bias;
    rectifiedscan.ranges = s->ranges;
    rectifiedscan.intensities = s->intensities;

//    rectifiedscan.ranges.reserve(s->ranges.size());
    for(unsigned int i = 0 ; i < s->ranges.size() ; i++)
    {
        rectifiedscan.ranges[i] = s->ranges[i] + bias;
    }

    rectifiedscan_pub.publish(rectifiedscan);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "HokuyoRectificator");
    HokuyoRectificator node;
    ros::spin();

    return 0;
}
