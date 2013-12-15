/*
 * ReplayOrder.cpp
 *
 *  Created on: 17 april 2012
 *      Author: RMO
 */

#include "ReplayOrder.hpp"
#include "control/orders/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

ReplayOrder::ReplayOrder(const OrderGoalConstPtr &goal, UbiquityMotionState currentMotionState, orders::config conf) :
        MotionOrder(goal, currentMotionState, conf)
{
    m_type = REPLAY;
    m_replayTime = 0;

    setConf(conf);
    m_replayDuration = goal->openloop_duration;

}

void ReplayOrder::switchRun(arp_math::Pose2D currentPosition)
{
    testTimeout();

    // test for DONE

    if (m_replayTime > m_replayDuration)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
        m_currentMode = MODE_DONE;
        return;
    }
}

ICRSpeed ReplayOrder::computeSpeed(UbiquityMotionState currentMotionState, UbiquityParams params, double dt)
{
    m_smoothLocNeeded = false;

    if (m_currentMode == MODE_DONE or m_currentMode == MODE_ERROR)
        return Twist2D(0, 0, 0);

    ICRSpeed speed_applied;
    double dt_applied;

    m_twistBuffer.removeICRSpeed(speed_applied, dt_applied);
    m_replayTime += dt_applied;

    //reverse the speed
    return ICRSpeed(-speed_applied.ro(), speed_applied.getICR());
}

