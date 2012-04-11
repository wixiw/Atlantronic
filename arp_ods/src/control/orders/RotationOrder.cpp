/*
 * RotationOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "RotationOrder.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace orders;

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

shared_ptr<MotionOrder>  RotationOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf )
{
    shared_ptr<RotationOrder> order(new RotationOrder());

    Pose2D begin;
    Pose2D end;

    begin.x(currentPose.x());
    begin.y(currentPose.y());
    begin.h(currentPose.h());
    order->setBeginPose(begin);

    end.x(currentPose.x());
    end.y(currentPose.y());
    end.h(goal->theta_des);
    order->setEndPose(end);

    order->setReverse(false);
    order->setPass(false);

    order->setDefaults(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void RotationOrder::switchInit(arp_math::Pose2D currentPosition)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();

    m_beginPose = currentPosition;
    m_endPose.x(m_beginPose.x());
    m_endPose.y(m_endPose.y());
    m_reverse = false;
    m_pass = false;

    ROS_INFO("switched MODE_INIT --> MODE_APPROACH because it's a RotationOrder");
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

