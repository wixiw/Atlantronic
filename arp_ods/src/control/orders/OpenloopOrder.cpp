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
using namespace std;
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

    Log(DEBUG) << ">> ---------------------------- NEW ORDER ----------------------";


    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void OpenloopOrder::switchRun(arp_math::Pose2D currentPosition)
{

    // test for DONE

    double t = getTime();
    double time_elapsed = t - m_initTime;

    if (m_initTime != -1 and time_elapsed > m_openloop_duration)
        {
            Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
            m_currentMode = MODE_DONE;
            return;
        }

    testTimeout();

}




Twist2D OpenloopOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{

    if (m_currentMode==MODE_DONE or m_currentMode==MODE_INIT)
        return Twist2D(0,0,0);

    Log(DEBUG) << ">> computespeed--------------------------";

    Log(DEBUG) << "mode: "<<m_currentMode;
    Log(DEBUG) << "---";

    //transfer of the twist to robot referential
    Twist2D v_correction_ref_init;
    v_correction_ref_init=m_openloop_twist.transport(m_cpoint.inverse());

    Log(DEBUG) << "time: "<<getTime();
    Log(DEBUG) << "dt: "<<dt;
    Log(DEBUG) << "---";
    Log(DEBUG) << "v_correction_ref_init: " << v_correction_ref_init.toString();
    Log(DEBUG) << "v_correction_ref_init.speedNorm(): "<<v_correction_ref_init.speedNorm();
    Log(DEBUG) << "---";

    // time to target
    double t_left = (m_initTime+m_openloop_duration)-getTime();

    //find the time when to begin deceleration
    double deceleration_time_lin=v_correction_ref_init.speedNorm()/m_conf.LIN_DEC;
    double deceleration_time_rot=fabs(v_correction_ref_init.vh())/m_conf.ANG_DEC;
    double deceleration_time=max(deceleration_time_lin,deceleration_time_rot);

    Log(DEBUG) << "deceleration_time_lin: " << deceleration_time_lin;
    Log(DEBUG) << "deceleration_time_rot: " << deceleration_time_rot;
    Log(DEBUG) << "deceleration_time: " << deceleration_time;
    Log(DEBUG) << "m_initTime: " << m_initTime;
    Log(DEBUG) << "m_openloop_duration: " << m_openloop_duration;
    Log(DEBUG) << "t_left: " << t_left;
    Log(DEBUG) << "---";

    //is the twist the nominal one or is it reduced because approaching end time ?
    Twist2D v_correction_ref;
    if (t_left<deceleration_time)
    {
        Log(DEBUG) << "decelerated twist";
    v_correction_ref=v_correction_ref_init*(t_left/deceleration_time);
    }
    else //deceleration has not begun
    {
        Log(DEBUG) << "nominal twist";
        v_correction_ref=v_correction_ref_init;
    }

    Log(DEBUG) << "v_correction_ref: " << v_correction_ref.toString();
    Log(DEBUG) << "v_correction_ref.speedNorm(): "<<v_correction_ref.speedNorm();
    Log(DEBUG) << "---";

    //saturation of twist: limit max linear speed/acc;  and max rotation speed/acc
    Twist2D v_correction_saturated;
    //v maxlin is the minimum of the max velocity and the old velocity + max acceleration

    // LINEAR (always positive)
    //limit due to acceleration and deceleration
    double vmaxlin_sataccdec=firstDerivateLimitation(v_correction_ref.speedNorm(), m_v_correction_old.speedNorm(), dt, -m_conf.LIN_DEC, m_conf.LIN_DEC);
    //selection of the smallest
    double vmaxlin=min(vmaxlin_sataccdec,m_conf.LIN_VEL_MAX);

    //ANGULAR (signed)
    double vmaxrot_sataccdec=fabs(firstDerivateLimitation(v_correction_ref.vh(), m_v_correction_old.vh(), dt, -m_conf.ANG_DEC, m_conf.ANG_DEC));
    double vmaxrot=min(vmaxrot_sataccdec,m_conf.ANG_VEL_MAX);

    //selection of the most saturating
    double satvlin;
    if (v_correction_ref.speedNorm()!=0.0)
        satvlin=v_correction_ref.speedNorm()/vmaxlin;
    else
        satvlin=0;

    double satvrot;
    if (fabs(v_correction_ref.vh())!=0.0)
        satvrot=fabs(v_correction_ref.vh())/vmaxrot;
    else
        satvrot=0;

    double sat=max(satvlin,max( satvrot,1.0));

    Log(DEBUG) << "vmaxlin_sataccdec : " << vmaxlin_sataccdec;
    Log(DEBUG) << "vmaxlin : " << vmaxlin;
    Log(DEBUG) << "vmaxrot : " << vmaxrot;
    Log(DEBUG) << "satvlin : " << satvlin;
    Log(DEBUG) << "satvrot : " << satvrot;
    Log(DEBUG) << "sat : " << sat;

    //application of the saturation
    v_correction_saturated = v_correction_ref * (1/sat);

    m_v_correction_old=v_correction_saturated;

    Log(DEBUG) << "v_correction_saturated: " << v_correction_saturated.toString();
    Log(DEBUG) << "v_correction_saturated.speedNorm(): "<<v_correction_saturated.speedNorm();
    Log(DEBUG) << "<< computespeed---------------------------";
    return v_correction_saturated;


}


