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
    Log(DEBUG) << ">>    creation d'un StayOrder    ";

    m_type = STAY;
    m_timeout=1000;

    Log(DEBUG) << "<<    creation d'un StayOrder    ";
}

void StayOrder::switchInit(arp_math::UbiquityMotionState currentMotionState)
{
    m_initialICRSpeed = currentMotionState.getSpeed();

    MotionOrder::switchInit(currentMotionState);
}


ICRSpeed StayOrder::computeSpeed(UbiquityMotionState currentMotionState, double dt)
{
    m_smoothLocNeeded = false;

    return ICRSpeed(0,m_initialICRSpeed.getICR());
}


