/*
 * RotationOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "RotationOrder.hpp"

namespace arp_ods
{

RotationOrder::RotationOrder()
{
    m_type = ROTATE;
    m_reverse = false;
    m_pass = false;
}

RotationOrder::RotationOrder(MotionOrder order):
        FantomOrder(order)
{
}

shared_ptr<MotionOrder>  RotationOrder::createOrder( const OrderGoalConstPtr &goal, Pose currentPose, order::config conf )
{
    shared_ptr<MotionOrder> order(new MotionOrder());

    Pose begin;
    Pose end;

    begin.x = currentPose.x;
    begin.y = currentPose.y;
    begin.theta = currentPose.theta;
    order->setBeginPose(begin);

    end.x = currentPose.x;
    end.y = currentPose.y;
    end.theta = currentPose.theta;
    order->setEndPose(end);

    order->setReverse(false);
    order->setPass(false);

    order->setDefaults(conf);

    return order;
}

void RotationOrder::switchInit(arp_core::Pose currentPosition)
{
    m_beginPose = currentPosition;
    m_endPose.x = m_beginPose.x;
    m_endPose.y = m_endPose.y;
    m_reverse = false;
    m_pass = false;

    ROS_INFO("switched to mode MODE_APPROACH from MODE_INIT because it's a RotationOrder");
    m_currentMode = MODE_APPROACH;
    return;
}

void RotationOrder::setReverse(bool reverse)
{
    m_reverse = false;
}

void RotationOrder::setPass(bool pass)
{
    m_pass = false;
}

}
