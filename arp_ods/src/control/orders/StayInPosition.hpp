/*
 * StayInPosition.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef STAYINPOSITION_HPP_
#define STAYINPOSITION_HPP_

#include "MotionOrder.hpp"

namespace arp_ods
{

/**
 *  The robot is asked to stay on the position when in MODE_INIT
 */
class StayInPosition: public arp_ods::MotionOrder
{
    public:
        StayInPosition();
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition);
};

}

#endif /* STAYINPOSITION_HPP_ */
