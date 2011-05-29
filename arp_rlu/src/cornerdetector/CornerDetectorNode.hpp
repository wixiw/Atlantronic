/*
 * CornerDetectorNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_CORNERDETECTORNODE_HPP_
#define _ARP_RLU_CORNERDETECTORNODE_HPP_

#include "CornerDetector.hpp"
#include <sensor_msgs/LaserScan.h>
#include <arp_rlu/DetectCorner.h>

class CornerDetectorNode
{
    public:
        CornerDetectorNode();
        ~CornerDetectorNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * The object which concentrate skilled code
         */
        arp_rlu::CornerDetector cd;

        arp_rlu::Scan scan;

        /**
         * Used to subscribe to "scan"
         */
        ros::Subscriber scan_sub;

        /**
         * Callback to computed the received scan
         */
        void scanCallback(sensor_msgs::LaserScanConstPtr scan);

        /**
         * Used to provide DetectCorner service
         */
        ros::ServiceServer detectcorner_srv;

        /**
         * Called when DtectCorner service is called
         * \returns success boolean
         */
        bool detectCornerCallback(arp_rlu::DetectCorner::Request& req, arp_rlu::DetectCorner::Response& res);

};

#endif /* _ARP_RLU_CORNERDETECTORNODE_HPP_ */
