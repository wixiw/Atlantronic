/*
 * ReplayOrder.cpp
 *
 *  Created on: 17 april 2012
 *      Author: RMO
 */

#include "ReplayOrder.hpp"
#include "ods_logger/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

ReplayOrder::ReplayOrder(const OrderGoalConstPtr &goal, UbiquityMotionState currentMotionState, UbiquityParams params) :
        MotionOrder(goal, currentMotionState, params)
{
    m_type = REPLAY;
    m_replayTime = 0;

    m_replayDuration = goal->openloop_duration;
    m_timeout=m_replayDuration+1.0;

    Log(DEBUG) << "replay de " << m_replayDuration;
    Log(DEBUG) << "timeout initialise a " << m_timeout;

}

void ReplayOrder::switchRun(arp_math::UbiquityMotionState currentMotionState)
{
    Log(DEBUG) << ">> switchRun " ;
    testTimeout();

    // test for DONE

    Log(DEBUG) << " if (m_replayTime > m_replayDuration) " ;
    Log(DEBUG) << " m_replayTime" <<m_replayTime;
    Log(DEBUG) << " m_replayDuration" <<m_replayDuration;
    if (m_replayTime > m_replayDuration)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
        m_currentMode = MODE_DONE;
        return;
    }

    Log(DEBUG) << "<< switchRun " ;
}

ICRSpeed ReplayOrder::computeSpeed(UbiquityMotionState currentMotionState, double dt)
{
    Log(DEBUG) << ">> computeSpeed " ;
    m_smoothLocNeeded = false;

    if (m_currentMode == MODE_DONE or m_currentMode == MODE_ERROR)
        return Twist2D(0, 0, 0);

    ICRSpeed speed_applied;
    double dt_applied;

    m_twistBuffer.removeICRSpeed(speed_applied, dt_applied);
    m_replayTime += dt_applied;

    Log(DEBUG) << "dt_applied "<<dt_applied ;
    Log(DEBUG) << "m_replayTime "<<m_replayTime ;

    Log(DEBUG) << ">> computeSpeed " ;
    Log(DEBUG) << "                " ;

    //reverse the speed
    return ICRSpeed(-speed_applied.ro(), speed_applied.getICR());
}

