/*
 * MotionOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "math/math.hpp"
#include "MotionOrder.hpp"
#include "orders/orders.h"

using namespace arp_core;
using namespace arp_math;

namespace arp_ods
{

namespace order
{
    shared_ptr < MotionOrder > defaultOrder(new MotionOrder());
}

MotionOrder::MotionOrder(const MotionOrder& order):
        arp_ods::ModeSelector()
{
    m_type = order.m_type;
    m_reverse = order.m_reverse;
    m_pass = order.m_pass;
    m_beginPose = order.m_beginPose;
    m_endPose = order.m_endPose;
    m_angleAccuracy = order.m_angleAccuracy;
    m_distanceAccurancy = order.m_distanceAccurancy;
    m_radiusApproachZone = order.m_radiusApproachZone;
    m_radiusInitZone = order.m_radiusInitZone;
    //m_id != m_id !!
}

MotionOrder::MotionOrder() :
    arp_ods::ModeSelector(), m_type(NO_ORDER), m_reverse(false)
{

}

Velocity MotionOrder::computeSpeed(arp_core::Pose currentPosition)
{
    Velocity v;
    v.linear = 0;
    v.angular = 0;
    return v;
}

Pose MotionOrder::reversePosition(Pose p)
{
    Pose ret;
    ret.x = p.x;
    ret.y = p.y;
    if( m_reverse )
    {
        ret.theta = normalizeAngle(p.theta + PI);
    }
    else
    {
        ret.theta = normalizeAngle(p.theta);
    }
    return ret;
}

shared_ptr<MotionOrder> MotionOrder::createOrder( const OrderGoalConstPtr &goal, Pose currentPose )
{
    shared_ptr<MotionOrder> order(new MotionOrder());

    Pose begin;
    Pose end;

    begin.x = currentPose.x;
    begin.y = currentPose.y;
    begin.theta = currentPose.theta;
    order->setBeginPose(begin);

    end.x = goal->x_des;
    end.y = goal->y_des;
    end.theta = goal->theta_des;
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    return order;
}

OrderType MotionOrder::getType() const
{
    return m_type;
}

std::string MotionOrder::getTypeString() const
{
    switch (m_type)
    {
        case NO_ORDER:
            return "NO_ORDER";
            break;
        case STAY_IN_POSITION:
            return "STAY_IN_POSITION";
            break;
        case TRANSLATE:
            return "TRANSLATE";
            break;
        case ROTATE:
            return "ROTATE";
            break;
        case FANTOM:
            return "FANTOM";
            break;
        default:
            return "ORDER_UNKNOW";
            break;
    }
}

void MotionOrder::setReverse(bool reverse)
{
    m_reverse = reverse;
}

void MotionOrder::setId(int id)
{
    m_id = id;
}

}

