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

ReplayOrder::ReplayOrder() :
    MotionOrder()
{
    m_type = REPLAY;
    m_replayTime=0;
    m_replayDuration=0;
}

ReplayOrder::ReplayOrder(MotionOrder order) :
    MotionOrder(order)
{
}


shared_ptr<MotionOrder> ReplayOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf  )
{
    shared_ptr<ReplayOrder> order(new ReplayOrder());

    order->setConf(conf);
    order->m_replayDuration=goal->openloop_duration;


    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void ReplayOrder::switchRun(arp_math::Pose2D currentPosition)
{
    testTimeout();

    // test for DONE


    if (m_replayTime  > m_replayDuration)
        {
            Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
            m_currentMode = MODE_DONE;
            return;
        }
}




Twist2D ReplayOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{
    if (m_currentMode==MODE_DONE or m_currentMode==MODE_ERROR)
        return Twist2D(0,0,0);

    Twist2D twist_applied;
    double dt_applied;

    m_twistBuffer.removeTwist(twist_applied,dt_applied);
    m_replayTime+=dt_applied;

    return twist_applied*(-1.0);
}


