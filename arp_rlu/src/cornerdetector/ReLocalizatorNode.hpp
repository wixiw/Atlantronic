/*
 * CornerDetectorNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_RELOCALIZATORNODE_HPP_
#define _ARP_RLU_RELOCALIZATORNODE_HPP_

#include <arp_rlu/DetectCorner.h>
#include <arp_rlu/EstimatePosition.h>

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
        bool estimatePositionCallback(arp_rlu::EstimatePosition::Request& req, arp_rlu::EstimatePosition::Response& res);



};
}

#endif /* _ARP_RLU_RELOCALIZATORNODE_HPP_ */
