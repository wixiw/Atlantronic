/*
 * MotionOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "math/math.hpp"
#include "MotionOrder.hpp"
#include "control/orders/orders.h"
#include "control/orders/Logger.hpp"


using namespace arp_core::log;
using namespace arp_math;
using namespace arp_ods;
using namespace orders;


// Displaying:
std::ostream& operator<<(std::ostream& os, const Mode& mode)
{
    switch (mode)
    {
        case MODE_INIT:
            return os << "MODE_INIT";
            break;
        case MODE_RUN:
            return os << "MODE_RUN";
            break;
        case MODE_DONE:
            return os << "MODE_DONE";
            break;
        case MODE_ERROR:
            return os << "MODE_ERROR";
            break;
        default:
            return os << "MODE_UNKNOWN";
            break;
    }
    return os << "MODE_UNKNOWN";
}

MotionOrder::MotionOrder(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
        orders::config conf)
{
    m_smoothLocNeeded=false;
    m_error_old=Pose2D(0,0,0);
    OTG=NULL;

    m_beginMotionState = UbiquityMotionState();
    m_endMotionState = UbiquityMotionState();
    m_pass = false;

    Log(DEBUG) << "m_currentMode " << m_currentMode;

    m_currentMode = MODE_INIT;

    Log(DEBUG) << "m_currentMode  " << m_currentMode;

    m_passTime = 0;
    // -1 is used to recognize non initialized time
    m_initTime = -1;
    m_approachTime = -1;

    setConf(conf);
}


MotionOrder::~MotionOrder()
{

}

OrderType MotionOrder::getType() const
{
    return m_type;
}

std::string MotionOrder::getTypeString() const
{
    switch (m_type)
    {
        case STAY:
            return "STAY";
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

void MotionOrder::resetMode()
{
    m_currentMode = MODE_INIT;
}

void MotionOrder::setConf(config conf)
{
    m_conf=conf;
}

void MotionOrder::switchInit(UbiquityMotionState currentMotionState)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();
    testTimeout();
    m_currentMode = MODE_RUN;
    m_runTime = getTime();

    Log(INFO) << "[" << m_endMotionState.toString()<< "] entered MODE_INIT at time " << m_initTime;
}

void MotionOrder::switchRun(UbiquityMotionState currentMotionState)
{

}


void MotionOrder::switchDone(UbiquityMotionState currentMotionState)
{

}

void MotionOrder::switchError(UbiquityMotionState currentMotionState)
{

}


void MotionOrder::switchMode(UbiquityMotionState currentMotionState)
{
    switch (m_currentMode)
    {
        case MODE_INIT:
            switchInit(currentMotionState);
            break;
        case MODE_RUN:
            switchRun(currentMotionState);
            break;
        case MODE_DONE:
            switchDone(currentMotionState);
            break;
        case MODE_ERROR:
            switchError(currentMotionState);
            break;
        default:
            break;
    }
}

double MotionOrder::getRemainingDistance(UbiquityMotionState currentMotionState)
{
    return currentMotionState.getPosition().distanceTo(m_endMotionState.getPosition());
}

double MotionOrder::getRemainingAngle(UbiquityMotionState currentMotionState)
{
    double e_theta = currentMotionState.getPosition().angleTo(m_endMotionState.getPosition());
    return e_theta;
}

double MotionOrder::getCoveredDistance(UbiquityMotionState currentMotionState)
{
    return currentMotionState.getPosition().distanceTo(m_beginMotionState.getPosition());
}

UbiquityMotionState MotionOrder::getBeginMotionState() const
{
    return m_beginMotionState;
}

UbiquityMotionState MotionOrder::getEndMotionState() const
{
    return m_endMotionState;
}

Pose2D MotionOrder::getCpoint() const
{
    return m_cpoint;
}

bool MotionOrder::getPass() const
{
    return m_pass;
}

Mode MotionOrder::getMode() const
{
    return m_currentMode;
}

void MotionOrder::setPass(bool pass)
{
    m_pass = pass;
}

void MotionOrder::setPassSpeed(double passSpeed)
{
    if( passSpeed <= 0.0 )
    {
        m_passSpeed = 0.1;
        Log(ERROR) << "MotionOrder::setPassSpeed(" << passSpeed << ") : Pass Speed is incorrect, should be positive";
    }
    else
        m_passSpeed = passSpeed;
}

void MotionOrder::setBeginMotionState(UbiquityMotionState beginMotionState)
{
    beginMotionState.getPosition().h(betweenMinusPiAndPlusPi(beginMotionState.getPosition().h()));
    this->m_beginMotionState = beginMotionState;
}

void MotionOrder::setEndMotionState(UbiquityMotionState endMotionState)
{
    endMotionState.getPosition().h(betweenMinusPiAndPlusPi(endMotionState.getPosition().h()));
    this->m_endMotionState = endMotionState;
}


void MotionOrder::setCpoint(Pose2D cpoint)
{
    cpoint.h(betweenMinusPiAndPlusPi(cpoint.h()));
    this->m_cpoint = cpoint;
}

void MotionOrder::testTimeout()
{
    double t = getTime();
    double time_elapsed = t - m_initTime;

    if (m_initTime != -1 and time_elapsed > m_conf.ORDER_TIMEOUT)
    {
        Log(INFO) << "switched from " << getMode() << " to MODE_ERROR because of dt=" << time_elapsed;
        m_currentMode = MODE_ERROR;
        return;
    }
}

