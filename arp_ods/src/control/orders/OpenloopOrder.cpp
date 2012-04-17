/*
 * OpenloopOrder.cpp
 *
 *  Created on: 17 april 2012
 *      Author: RMO
 */

#include "OpenloopOrder.hpp"
#include "control/orders/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

OpenloopOrder::OpenloopOrder() :
    MotionOrder()
{
    m_type = OPENLOOP;
    m_v_correction_old=Twist2D(0,0,0);


}

OpenloopOrder::OpenloopOrder(MotionOrder order) :
    MotionOrder(order)
{
}


shared_ptr<MotionOrder> OpenloopOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf  )
{
    shared_ptr<OpenloopOrder> order(new OpenloopOrder());

    Pose2D cpoint;

    cpoint.x(goal->x_cpoint);
    cpoint.y(goal->y_cpoint);
    cpoint.h(goal->theta_cpoint);
    order->setCpoint(cpoint);

    order->m_openloop_twist=Twist2D(goal->x_speed,goal->y_speed,goal->theta_speed);
    order->m_openloop_duration=goal->openloop_duration;

    order->setConf(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void OpenloopOrder::switchRun(arp_math::Pose2D currentPosition)
{

    // test for DONE

    double t = getTime();
    double dt = t - m_initTime;

    if (m_initTime != -1 and dt > m_openloop_duration)
        {
            Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
            m_currentMode = MODE_DONE;
            return;
        }

    testTimeout();

}




Twist2D OpenloopOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{

    //transfer of the twist to robot referential
    Twist2D v_correction_ref;
    v_correction_ref=m_openloop_twist.transport(m_cpoint.inverse());

    //saturation of twist: limit max linear speed/acc;  and max rotation speed/acc
    Twist2D v_correction_saturated;
    double vmaxlin=std::min(m_conf.LIN_VEL_MAX,m_v_correction_old.speedNorm()+dt*m_conf.LIN_DEC);
    double vmaxrot=std::min(m_conf.ANG_VEL_MAX,std::fabs(m_v_correction_old.vh())+dt*m_conf.ANG_DEC);
    double satvlin=v_correction_ref.speedNorm()/vmaxlin;
    double satvrot=std::fabs(v_correction_ref.vh())/vmaxrot;
    double sat=std::max(satvlin,std::max( satvrot,1.0));
    v_correction_saturated = v_correction_ref * (1/sat);

    m_v_correction_old=v_correction_saturated;


    return v_correction_saturated;

}


