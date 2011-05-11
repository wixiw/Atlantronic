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

using namespace arp_core;
using namespace sensor_msgs;

namespace arp_rlu
{
	class ObstacleDetector
	{
	public:
		ObstacleDetector();

		void go();

	protected:
	    /**
	    * NodeHandle on associated node
	    */
	    ros::NodeHandle 	nh;

	    /**
	    * Used to subscribe to "scan"
	    */
	    ros::Subscriber    	scan_sub;

	    /**
	    * Used to publish on "obstacle"
	    */
	    ros::Publisher     	obs_pub;

	    /**
	     * Callback to computed the received scan an d publish obstacle value
	     */
	    void scanCallback(LaserScanConstPtr scan);

        /**
         * these constants are initialized by ros parameters. see .launch for explananations
         */
        double DETECTION_THRESHOLD;
	};
}

#endif /* OBSTACLEDETECTOR_HPP_ */
