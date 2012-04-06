/*
 * StayInPosition.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "StayInPosition.hpp"

namespace arp_ods
{

StayInPosition::StayInPosition() :
    MotionOrder()
{
    m_type = STAY_IN_POSITION;
}

arp_math::Twist2D StayInPosition::computeSpeed(arp_math::Pose2D currentPosition)
{
    arp_math::Twist2D v;
    return v;
}

}
