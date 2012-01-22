/*
 * DetectedCircle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_
#define _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_

#include <math/math.hpp>
#include "DetectedObject.hpp"
#include "Circle.hpp"

namespace arp_rlu
{

namespace lsl
{

class DetectedCircle : public Circle, public DetectedObject
{
    public:
        DetectedCircle();
        ~DetectedCircle();


    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_ */
