/*
 * CornerDetectorNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_RELOCALIZATORNODE_HPP_
#define _ARP_RLU_RELOCALIZATORNODE_HPP_

#include "ReLocalizator.hpp"

#include <tf/transform_listener.h>

#include <arp_rlu/DetectCorner.h>
#include <arp_rlu/EstimatePosition.h>

#include <string>
#include <vector>

namespace arp_rlu
{

class ReLocalizatorNode
{
    public:
        ReLocalizatorNode();
        ~ReLocalizatorNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * Used to provide DetectCorner service
         */
        ros::ServiceServer estimatePosition_srv;

        /**
         * Called when DtectCorner service is called
         * \returns success boolean
         */
        bool
        estimatePositionCallback(arp_rlu::EstimatePosition::Request& req, arp_rlu::EstimatePosition::Response& res);

        /**
         * used to call spawn (reset) service of Laserator
         */
        ros::ServiceClient cornerdetection_client_;

        ReLocalizator rl;

        /**
         * Ecoute les tf
         */
        tf::TransformListener m_tfListener;

        /**
         * Name of the tf of the robot
         */
        std::string m_baseFrameName;

        /**
         * Name of the front laser
         */
        std::string m_frontLaserFrameName;

        /**
         * Tf from Base to front obstacle laser
         */
        tf::StampedTransform m_baseToFrontLaser;

};
}

#endif /* _ARP_RLU_RELOCALIZATORNODE_HPP_ */
