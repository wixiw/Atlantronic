/*
 * DetectedCircle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_
#define _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_

#include <math/core>

#include <LSL/objects/DetectedObject.hpp>
#include <LSL/objects/Circle.hpp>

namespace arp_rlu
{

namespace lsl
{
/** \ingroup lsl
 * \nonstableyet
 *
 * \class DetectedCircle
 *
 * \brief DetectedCircle correspond à la fois à un DetectedObject et à un Circle.
 */
class DetectedCircle : public Circle, public DetectedObject
{
    public:
        /**
         * Constructeur par défault qui appelle les constructeurs par défault de Circle et de DetectedObject.
         */
        DetectedCircle();

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_DETECTEDCIRCLE_HPP_ */
