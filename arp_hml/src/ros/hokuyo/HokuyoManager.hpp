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
        bool scanReceived;
        ros::WallTime last_run_time_;
        ros::WallTime last_kill_time_;

        /**
         * Used to subscribe to "scan"
         */
        ros::Subscriber scan_sub;

        /**
         * Callback to computed the received scan an d publish obstacle value
         */
        void scanCallback(LaserScanConstPtr scan);

        void spin();

        void shutDown();
};
}

#endif /* _ARD_HML_HOKUYOMANAGER_HPP_ */
