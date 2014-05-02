/*
 * OpenloopOrder.cpp
 *
 *  Created on: 17 april 2012
 *      Author: RMO
 */

#include "OpenloopOrder.hpp"
#include "ods_logger/Logger.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace arp_time;
using namespace orders;
using namespace std;

OpenloopOrder::OpenloopOrder(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
        UbiquityParams params) :
        MotionOrder(goal, currentMotionState, params)
{
    m_type = OPENLOOP;

    m_v_correction_old = Twist2D(0, 0, 0);

    Pose2D cpoint;

    cpoint.x(goal->x_cpoint);
    cpoint.y(goal->y_cpoint);
    cpoint.h(goal->theta_cpoint);
    setCpoint(cpoint);

    m_openloop_twist = Twist2D(goal->x_speed, goal->y_speed, goal->theta_speed);
    if (goal->openloop_duration < MAX_OPENLOOP_TIME)
        m_openloop_duration = goal->openloop_duration;
    else
        m_openloop_duration = MAX_OPENLOOP_TIME;

    m_timeout = m_openloop_duration + 1.0;

    Log(INFO) << getTypeString() << " with Twist "<<m_openloop_twist.toString()<< "and time "<<m_openloop_duration<<"at cpoint "<<cpoint.toString();

}

void OpenloopOrder::switchRun(UbiquityMotionState currentMotionState)
{

    // test for DONE

    ArdAbsoluteTime t = getAbsoluteTime();
    ArdTimeDelta time_elapsed = getTimeDelta(m_initTime, t);

    if (m_initTime != -1 and time_elapsed > m_openloop_duration)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
        m_currentMode = MODE_DONE;
        return;
    }

    testTimeout();

}

ICRSpeed OpenloopOrder::computeSpeed(UbiquityMotionState currentMotionState, ArdTimeDelta dt)
{
    Log(DEBUG) << ">>computeSpeed  ";
    m_smoothLocNeeded = false;

    if (m_currentMode == MODE_DONE or m_currentMode == MODE_INIT or m_currentMode == MODE_ERROR)
        return ICRSpeed(0, 0, 0);

    ICRSpeed ICRSpeed_correction_ref_init(m_openloop_twist);
    ICRSpeed_correction_ref_init=ICRSpeed_correction_ref_init.getNormalizedRep();
    double ro_ref_init=ICRSpeed_correction_ref_init.ro();

    Log(DEBUG) << "ro_ref_init "<<ro_ref_init;

    ArdTimeDelta acc_time=ro_ref_init/m_params.getMaxRobotAccel();
    ArdTimeDelta startDec = m_openloop_duration-acc_time;
    ArdTimeDelta curTime=getAbsoluteTime()-m_initTime;

    Log(DEBUG) << "acc_time "<<acc_time;
    Log(DEBUG) << "startDec "<<startDec;
    Log(DEBUG) << "curTime "<<curTime;


    double roAccDec=0;

    if (curTime>=0 and curTime<acc_time)
        {
        roAccDec=ro_ref_init/acc_time*curTime;
        Log(DEBUG) << "-> acc  ";
        }

    if (curTime>=acc_time and curTime<startDec)
        {
        roAccDec=ro_ref_init;
        Log(DEBUG) << "-> plateau  ";
        }

    if (curTime>=startDec and curTime<m_openloop_duration)
        {
        roAccDec=ro_ref_init-ro_ref_init/acc_time*(curTime-startDec);
        Log(DEBUG) << "-> dec  ";
        }

    Log(DEBUG) << "roAccDec "<<roAccDec;

    ICRSpeed ICRSpeedCor=ICRSpeed_correction_ref_init;
    ICRSpeedCor.ro(roAccDec);

    Log(DEBUG) << "ICRSpeedCor "<<ICRSpeedCor;

    Log(DEBUG) << "<<computeSpeed  ";

    return ICRSpeedCor;

}

