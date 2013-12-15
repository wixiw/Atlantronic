/*
 * StayInPosition.cpp
 *
 *  Created on: 15 december 2013
 *      Author: RMO
 */

#include "StayOrder.hpp"
#include "control/orders/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

StayOrder::StayOrder(const OrderGoalConstPtr &goal,UbiquityMotionState currentMotionState, orders::config conf  ) :
    MotionOrder(goal,currentMotionState,conf)
{
    m_type = STAY;
}



ICRSpeed StayOrder::computeSpeed(UbiquityMotionState currentMotionState,UbiquityParams params, double dt)
{
    m_smoothLocNeeded = false;
    return ICRSpeed(0,0,0);
}


