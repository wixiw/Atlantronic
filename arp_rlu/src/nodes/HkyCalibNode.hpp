/*
 * HkyCalibNode.hpp
 *
 *  Created on: 13 avril 2012
 *      Author: ard
 */

#ifndef _ARP_RLU_HKYCALIBNODE_HPP_
#define _ARP_RLU_HKYCALIBNODE_HPP_

#include <KFL/BeaconDetector.hpp>
#include <sensor_msgs/LaserScan.h>
#include <arp_rlu/HkyCalib.h>

#include <string>
#include <vector>

namespace arp_rlu
{

class HkyCalibNode
{
    public:
        HkyCalibNode();
        ~HkyCalibNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

        /**
         * Used to subscribe to "scan"
         */
        ros::Subscriber     m_scanSubscriber;

        /**
         * Used to provide HkyCalib service
         */
        ros::ServiceServer calib_srv;

        /**
         * Callback to computed the received scan an d publish obstacle value
         */
        void scanCallback(sensor_msgs::LaserScanConstPtr scan);

        /**
         * Called when service is called
         * \returns success boolean
         */
        bool
        serviceCallback(HkyCalib::Request& req, HkyCalib::Response& res);

        lsl::LaserScan lastScan;

        kfl::BeaconDetector beaconDetector;

};
}

#endif /* _ARP_RLU_HKYCALIBNODE_HPP_ */
