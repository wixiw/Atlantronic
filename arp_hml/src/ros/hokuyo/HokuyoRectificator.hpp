/*
 * HokuyoRectificator.hpp
 *
 *  Created on: 26 mai 2011
 *      Author: Boris
 */

#ifndef _ARD_HML_HOKUYORECTIFICATOR_HPP_
#define _ARD_HML_HOKUYORECTIFICATOR_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace sensor_msgs;

namespace arp_hml
{
class HokuyoRectificator
{
    public:
        HokuyoRectificator();
        ~HokuyoRectificator();

        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * Used to subscribe to "scan"
         */
        ros::Subscriber scan_sub;

        /**
         * Used to publish on "rectifiedscan"
         */
        ros::Publisher rectifiedscan_pub;

        /**
         * Callback to computed the received scan
         */
        void scanCallback(sensor_msgs::LaserScanConstPtr scan);

        double bias;
};
}

#endif /* _ARD_HML_HOKUYORECTIFICATOR_HPP_ */
