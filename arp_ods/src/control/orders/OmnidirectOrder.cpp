/*
 * OmnidirectOrder.cpp
 *
 *  Created on: 10 april 2012
 *      Author: RMO
 */

#include "OmnidirectOrder.hpp"

using namespace arp_math;

namespace arp_ods
{

OmnidirectOrder::OmnidirectOrder() :
    MotionOrder()
{
    m_type = OMNIDIRECT;

}

OmnidirectOrder::OmnidirectOrder(MotionOrder order) :
    MotionOrder(order)
{
}

void OmnidirectOrder::setDefaults(order::config conf)
{
    MotionOrder::setDefaults(conf);

}

shared_ptr<MotionOrder> OmnidirectOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, order::config conf  )
{
    shared_ptr<OmnidirectOrder> order(new OmnidirectOrder());

    Pose2D begin;
    Pose2D end;

    begin.x(currentPose.x());
    begin.y(currentPose.y());
    begin.h(currentPose.h());
    order->setBeginPose(begin);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    order->setDefaults(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}



Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{
    Twist2D v;

    v.vx(0);
    v.vy(0.1);
    v.vh(0);
    return v;
}

}

