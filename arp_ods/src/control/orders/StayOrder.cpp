/*
 * StayInPosition.cpp
 *
 *  Created on: 15 december 2013
 *      Author: RMO
 */

#include "StayOrder.hpp"
#include "ods_logger/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

StayOrder::StayOrder(const OrderGoalConstPtr &goal,UbiquityMotionState currentMotionState, UbiquityParams params ) :
    MotionOrder(goal,currentMotionState,params)
{
    Log(DEBUG) << ">>    creation d'un StayOrder    ";

    m_type = STAY;
    m_timeout=1000;

    Log(DEBUG) << "<<    creation d'un StayOrder    ";
}

void StayOrder::switchInit(arp_math::UbiquityMotionState currentMotionState)
{
    m_initialICRSpeed = currentMotionState.getSpeed();

    MotionOrder::switchInit(currentMotionState);
}


ICRSpeed StayOrder::computeSpeed(UbiquityMotionState currentMotionState, double dt)
{
    m_smoothLocNeeded = false;

    /*
    Log(DEBUG) << "m_initialICRSpeed.ro()                                "<<m_initialICRSpeed.ro();
    Log(DEBUG) << "dt                                                    "<< dt;
    Log(DEBUG) << "m_params.getMaxRobotAccel()                           "<<m_params.getMaxRobotAccel();
    Log(DEBUG) << "m_initialICRSpeed.ro()-dt*m_params.getMaxRobotAccel() "<< m_initialICRSpeed.ro()-dt*m_params.getMaxRobotAccel();
*/

    //m_initialICRSpeed.ro(max(m_initialICRSpeed.ro()-dt*m_params.getMaxRobotAccel(),0.0));
    //TODO sorry pour l'accel en dure, c'est a cause du createDefaultOrder de la fatcoriy, dont ROSinterface a besoin quand il se lance
    m_initialICRSpeed.ro(max(m_initialICRSpeed.ro()-dt*1.0,0.0));

    return m_initialICRSpeed;
}


