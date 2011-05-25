/*
 * ObstacleDetector.cpp
 *
 *  Created on: 11 mai 2011
 *      Author: wla
 */

#include "ros/ros.h"
#include "ObstacleDetector.hpp"

using namespace arp_rlu;
using namespace sensor_msgs;

ObstacleDetector::ObstacleDetector():
		nh()
{
	scan_sub = nh.subscribe("scan", 1, &ObstacleDetector::scanCallback, this);
	obs_pub = nh.advertise<Obstacle>("obstacle", 1);

	if( !ros::NodeHandle("~").getParam("DETECTION_THRESHOLD", DETECTION_THRESHOLD) )
	{
		ROS_WARN("did not found DETECTION_THRESHOLD, taking 50cm");
		DETECTION_THRESHOLD = 0.50;
	}

}

void ObstacleDetector::go()
{
	ros::spin();
}

void ObstacleDetector::scanCallback(LaserScanConstPtr scan)
{
	Obstacle obs;
	obs.detected = 0;
	float d = scan->range_max;

	//ROS_WARN("min : %f max : %f",scan->range_min, scan->range_max);
	for( unsigned int i=0 ; i!= scan->ranges.size() ; i++)
	{
		if( scan->ranges[i] < d && scan->range_min < scan->ranges[i] )
		{
			d = scan->ranges[i];
		}

	}

	//ROS_WARN("detected : %f",d);

	if( d < DETECTION_THRESHOLD && d > scan->range_min)
	{
		obs.detected = d;
		obs_pub.publish(obs);
	}
}
