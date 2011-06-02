/*
 * ObjectFinderNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_CHESSBOARDREADER_HPP_
#define _ARP_RLU_CHESSBOARDREADER_HPP_

#include "ObjectFinder.hpp"
#include "KnownObject.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <arp_rlu/FindRoyalFamily.h>

#include <string>
#include <vector>

namespace arp_rlu
{

class ChessboardReader
{
    public:
        ChessboardReader();
        ~ChessboardReader();

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
         * Used to provide FindRoyalFamily service
         */
        ros::ServiceServer findroyalfamily_srv;

        /**
         * Called when FindRoyalFamily service is called
         * \returns success boolean
         */
        bool findRoyalFamilyCallback(arp_rlu::FindRoyalFamily::Request& req, arp_rlu::FindRoyalFamily::Response& res);


        std::vector<KnownObject> objects;

};
}

#endif /* _ARP_RLU_CHESSBOARDREADER_HPP_ */
