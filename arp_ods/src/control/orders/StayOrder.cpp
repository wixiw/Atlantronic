/*
 * StayInPosition.cpp
 *
 *  Created on: 15 december 2013
 *      Author: RMO
 */

#include "StayOrder.hpp"
#include "ods_logger/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

StayOrder::StayOrder(const OrderGoalConstPtr &goal,UbiquityMotionState currentMotionState, UbiquityParams params ) :
    MotionOrder(goal,currentMotionState,params)
{
    m_type = STAY;
    m_timeout=1000;
}



ICRSpeed StayOrder::computeSpeed(UbiquityMotionState currentMotionState, double dt)
{
    m_smoothLocNeeded = false;
    return ICRSpeed(0,0,0);
}


