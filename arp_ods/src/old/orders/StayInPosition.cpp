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

Velocity StayInPosition::computeSpeed(arp_core::Pose currentPosition)
{
    Velocity v;
    return v;
}

}
