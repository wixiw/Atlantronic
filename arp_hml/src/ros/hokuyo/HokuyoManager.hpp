/*
 * HokuyoManager.hpp
 *
 *  Created on: 26 mai 2011
 *      Author: Boris
 */

#ifndef _ARD_HML_HOKUYOMANAGER_HPP_
#define _ARD_HML_HOKUYOMANAGER_HPP_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

using namespace sensor_msgs;

namespace arp_hml
{
class HokuyoManager
{
    public:
        HokuyoManager(ros::NodeHandle &nh);
        ~HokuyoManager();

        /**
         * Used to subscribe to "Protokrot/emergency_stop"
         */
        ros::Subscriber emergency_sub;

        void auCallback(const std_msgs::BoolConstPtr& au);

        bool emergency;
        bool isRunning;

        void spin();
        void shutDown();
};
}

#endif /* _ARD_HML_HOKUYOMANAGER_HPP_ */
