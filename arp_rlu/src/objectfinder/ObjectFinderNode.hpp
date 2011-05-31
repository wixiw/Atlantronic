/*
 * ObjectFinderNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_OBJECTFINDERNODE_HPP_
#define _ARP_RLU_OBJECTFINDERNODE_HPP_

#include "ObjectFinder.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <arp_rlu/FindObjects.h>

#include <string>
#include <vector>

namespace arp_rlu
{

class ObjectFinderNode
{
    public:
        ObjectFinderNode();
        ~ObjectFinderNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * The object which concentrate skilled code
         */
        arp_rlu::ObjectFinder objf;

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
        ros::ServiceServer findobjects_srv;

        /**
         * Called when DtectCorner service is called
         * \returns success boolean
         */
        bool findobjectsCallback(arp_rlu::FindObjects::Request& req, arp_rlu::FindObjects::Response& res);

        bool reverse_scan;

};
}

#endif /* _ARP_RLU_OBJECTFINDERNODE_HPP_ */
