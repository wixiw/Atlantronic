/*
 * ObstacleDetector.cpp
 *
 *  Created on: 11 mai 2011
 *      Author: wla
 */

#include "ros/ros.h"
#include "ObstacleDetector.hpp"

using namespace arp_rlu;
using namespace arp_core;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace geometry_msgs;

ObstacleDetector::ObstacleDetector()
{
    ros::NodeHandle nh = ros::NodeHandle();

	m_scanSubscriber = nh.subscribe("top_scan", 1, &ObstacleDetector::scanCallback, this);
	m_poseSubscriber =  nh.subscribe("Localizator/odomRos", 1, &ObstacleDetector::poseCallback, this);
	m_rearObstacleSubscriber = nh.subscribe("Protokrot/rear_obstacle", 1, &ObstacleDetector::rearObstacleCallback, this);

	m_rearObstaclePublisher = nh.advertise<Obstacle>("ObstacleDetector/rear_obstacle", 1);
	m_frontObstaclePublisher = nh.advertise<Obstacle>("ObstacleDetector/front_obstacle", 1);

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
		obs.detected = 1;
		//TODO WLA mettre la position
	}
	m_frontObstaclePublisher.publish(obs);
}

void ObstacleDetector::poseCallback(OdometryConstPtr odom)
{
    m_currentPose = odom->pose.pose;
}

void ObstacleDetector::rearObstacleCallback(BoolConstPtr detected)
{
    Obstacle obstacle;
    obstacle.detected = detected->data;
    //TODO WLA mettre la position de la détection
    m_rearObstaclePublisher.publish(obstacle);
}
