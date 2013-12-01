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
    m_passSpeed = order.m_passSpeed;
    m_beginMotionState = order.m_beginMotionState;
    m_endMotionState = order.m_endMotionState;
    m_conf=order.m_conf;
    m_openloop_twist=order.m_openloop_twist;
    m_openloop_duration=order.m_openloop_duration;
    m_smoothLocNeeded=false;
    m_error_old=Pose2D(0,0,0);
    OTG=NULL;
}

MotionOrder::MotionOrder() :
   ModeSelector(), m_type(NO_ORDER)
{

}

MotionOrder::~MotionOrder()
{

}

ICRSpeed MotionOrder::computeSpeed(UbiquityMotionState currentMotionState,UbiquityParams params, double dt)
{
    m_smoothLocNeeded = false;

    ICRSpeed defaultNullSpeedCmd(0,0,0);
    return defaultNullSpeedCmd;
}

shared_ptr<MotionOrder> MotionOrder::createOrder( const OrderGoalConstPtr &goal, UbiquityMotionState currentMotionState, orders::config conf )
{
    shared_ptr<MotionOrder> order(new MotionOrder());

    order->setBeginMotionState(currentMotionState);

    Pose2D end;
    UbiquityMotionState endMotionState;
    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    endMotionState.setPosition(end);
    //TODO mapper la vitesse
    order->setEndMotionState(endMotionState);

    order->setPass(goal->passe);
    order->setPassSpeed(goal->passe_speed);

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

void MotionOrder::setICRSpeedBuffer(ICRSpeedBuffer twistBuffer )
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
