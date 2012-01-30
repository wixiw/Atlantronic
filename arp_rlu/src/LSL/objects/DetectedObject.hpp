/*
 * DetectedObject.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_DETECTEDOBJECT_HPP_
#define _ARP_RLU_LSL_DETECTEDOBJECT_HPP_

#include <math/core>

#include <LSL/LaserScan.hpp>

namespace arp_rlu
{

namespace lsl
{

class DetectedObject
{
    public:
        DetectedObject();
        ~DetectedObject();

        void setScan(lsl::LaserScan);

        arp_math::Vector2 getCartesianMean() const;
        arp_math::Vector2 getCartesianStddev() const;
        double getApparentRange() const;
        double getApparentTheta() const;


    protected:
        lsl::LaserScan associatedScan;
        double apparentRange;
        double apparentTheta;
        arp_math::Vector2 cartMean;
        arp_math::Vector2 cartStddev;

        void computeStatistics();
};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDOBJECT_HPP_ */
