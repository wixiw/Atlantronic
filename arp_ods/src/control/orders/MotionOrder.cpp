/*
 * MotionOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "math/math.hpp"
#include "MotionOrder.hpp"
#include "control/orders/orders.h"


using namespace arp_math;
using namespace arp_ods;
using namespace orders;
namespace arp_ods{
    namespace orders
{
    shared_ptr < MotionOrder > defaultOrder(new MotionOrder());
}}

MotionOrder::MotionOrder(const MotionOrder& order):
        ModeSelector()
{
    m_type = order.m_type;
    m_pass = order.m_pass;
    m_beginPose = order.m_beginPose;
    m_endPose = order.m_endPose;
    m_conf=order.m_conf;
    m_openloop_twist=order.m_openloop_twist;
    m_openloop_duration=order.m_openloop_duration;
    m_smoothLocNeeded=false;
    m_error_old=Pose2D(0,0,0);
    OTG=NULL;

    attrGain=0.03;
    //m_id != m_id !!
}

MotionOrder::MotionOrder() :
   ModeSelector(), m_type(NO_ORDER)
{

}

Twist2D MotionOrder::computeSpeed(Pose2D currentPosition,MotorState motorState,UbiquityParams params, double dt)
{
    m_smoothLocNeeded = false;

    Twist2D v;
    v.vx(0);
    v.vy(0);
    v.vh(0);
    return v;
}

shared_ptr<MotionOrder> MotionOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf )
{
    shared_ptr<MotionOrder> order(new MotionOrder());

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

    order->setPass(goal->passe);

    order->setConf(conf);

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
        case OMNIDIRECT:
            return "OMNIDIRECT";
            break;
        case OMNIDIRECT2:
            return "OMNIDIRECT2";
            break;
        case OPENLOOP:
            return "OPENLOOP";
            break;
        case REPLAY:
            return "REPLAY";
            break;

        default:
            return "ORDER_UNKNOW";
            break;
    }
    return "ERROR";
}

void MotionOrder::setId(int id)
{
    m_id = id;
}

void MotionOrder::setTwistBuffer(TwistBuffer twistBuffer )
{
    m_twistBuffer = twistBuffer;
}

void MotionOrder::setOTG(OnlineTrajectoryGenerator * OTG_)
{
    OTG = OTG_;
}

void MotionOrder::setVmax(double vmax)
{
if (vmax>=0.0)
    m_vmax_asked=vmax;
else
    m_vmax_asked=m_conf.LIN_VEL_MAX;

}
