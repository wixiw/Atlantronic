/*
 * ObstacleDetector.hpp
 *
 *  Created on: 11 mai 2011
 *      Author: wla
 */

#ifndef OBSTACLEDETECTOR_HPP_
#define OBSTACLEDETECTOR_HPP_

#include "ros/ros.h"
#include <arp_core/Obstacle.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

using namespace arp_core;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

namespace arp_rlu
{
	class ObstacleDetector
	{
	public:
		ObstacleDetector();

		void go();

	protected:
	    /**
	    * Used to subscribe to "scan"
	    */
	    ros::Subscriber    	m_scanSubscriber;

        /**
        * Used to subscribe to "Localizator/odomRos"
        */
        ros::Subscriber     m_poseSubscriber;

        /**
        * Used to subscribe to "/Protokrot/rear_obstacle"
        */
        ros::Subscriber     m_rearObstacleSubscriber;

	    /**
	    * Used to publish on "obstacle"
	    */
	    ros::Publisher     	m_frontObstaclePublisher;

        /**
        * Used to publish on "obstacle"
        */
        ros::Publisher      m_rearObstaclePublisher;

	    /** */
	    geometry_msgs::Pose m_currentPose;

	    /**
	     * Callback to computed the received scan an d publish obstacle value
	     */
	    void scanCallback(LaserScanConstPtr scan);

	    /**
	     * Callback to computed received posiiton
	     */
	    void poseCallback(OdometryConstPtr scan);

	    /**
	     * Callback to computed received rear obstacle
	     */
	    void rearObstacleCallback(BoolConstPtr scan);

        /**
         * these constants are initialized by ros parameters. see .launch for explananations
         */
        double DETECTION_THRESHOLD;
	};
}

#endif /* OBSTACLEDETECTOR_HPP_ */
