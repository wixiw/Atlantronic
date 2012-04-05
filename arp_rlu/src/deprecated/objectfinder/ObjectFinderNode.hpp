/*
 * ObjectFinderNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_OBJECTFINDERNODE_HPP_
#define _ARP_RLU_OBJECTFINDERNODE_HPP_

#include "ObjectFinder.hpp"
#include "KnownObject.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
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
         * Publisher for rviz display
         */
        ros::Publisher m_towerPublisher;
        ros::Publisher m_robotPublisher;
        ros::Publisher m_figurePublisher;
        ros::Publisher m_ufoPublisher;

        sensor_msgs::PointCloud m_towerPointCloud;
        sensor_msgs::PointCloud m_robotPointCloud;
        sensor_msgs::PointCloud m_figurePointCloud;
        sensor_msgs::PointCloud m_ufoPointCloud;

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

        std::vector<KnownObject> objects;

        /**
         * Add an object to data published for Rviz. It finds the correcponsding point cloud topic
         * related to the obj.type
         * @param obj object to add
         */
        void recordObjectForRviz(KnownObject obj);

        /**
         * Publish point cloud topics for Rviz display
         */
        void publishForRviz();

        /**
         * append scan to the point cloud c
         * @param s : scan to add to the point cloud
         * @param c : point cloud receiving the object
         */
        void addToPointCloud(Scan s, sensor_msgs::PointCloud c);

};
}

#endif /* _ARP_RLU_OBJECTFINDERNODE_HPP_ */
