/*
 * StayInPosition.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef STAYINPOSITION_HPP_
#define STAYINPOSITION_HPP_

#include "MotionOrder.hpp"

namespace arp_ods{ namespace orders
{

/**
 *  The robot is asked to stay on the position when in MODE_INIT
 */
class StayOrder: public MotionOrder
{
    public:
        StayOrder();
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition);
};

}}

#endif /* STAYINPOSITION_HPP_ */
