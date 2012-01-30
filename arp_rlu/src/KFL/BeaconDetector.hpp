/*
 * BeaconDetector.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BEACONDETECTOR_HPP_
#define _ARP_RLU_KFL_BEACONDETECTOR_HPP_

#include <vector>
#include <utility>

#include <math/core>

#include "LSL/LaserScan.hpp"
#include "LSL/objects/Circle.hpp"
#include "LSL/filters/MedianFilter.hpp"
#include "LSL/filters/PolarCrop.hpp"
#include "LSL/filters/CartesianCrop.hpp"
#include "LSL/filters/PolarSegment.hpp"



namespace arp_rlu
{

namespace kfl
{

class BeaconDetector
{
    public:
        class Params
        {
        public:
            Params();
            std::string getInfo();

            lsl::MedianFilter::Params mfp;
            lsl::PolarCrop::Params pcp;
            lsl::CartesianCrop::Params ccp;
            lsl::PolarSegment::Params psp;
        };

    public:
        BeaconDetector();
        ~BeaconDetector();

        bool process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh);
        void setRefecencedBeacons(std::vector<lsl::Circle> beacons);
        bool getBeacon(double t, std::pair<lsl::Circle, Eigen::Vector3d > & p);
        void setParams(kfl::BeaconDetector::Params);

    protected:
        void reset();

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BEACONDETECTOR_HPP_ */
