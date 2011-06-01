/*
 * LaserCalibratorNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_LASERCALIBRATORNODE_HPP_
#define _ARP_RLU_LASERCALIBRATORNODE_HPP_

#include <arp_rlu/DetectCorner.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>

namespace arp_rlu
{

class LaserCalibratorNode
{
    public:
        LaserCalibratorNode();
        ~LaserCalibratorNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * Used to provide DetectCorner service
         */
        ros::ServiceServer calib_srv;

        /**
         * Called when DtectCorner service is called
         * \returns success boolean
         */
        bool
        calibLaserCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        /**
         * used to call spawn (reset) service of Laserator
         */
        ros::ServiceClient cornerdetection_client_;

};
}

#endif /* _ARP_RLU_LASERCALIBRATORNODE_HPP_ */
