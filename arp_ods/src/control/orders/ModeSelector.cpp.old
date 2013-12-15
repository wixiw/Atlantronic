/*
 * ModeSelector.cpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */
#include "ModeSelector.hpp"
#include "control/orders/Logger.hpp"
#include <math/core>
#include <ros/ros.h>


using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

// Displaying:
std::ostream& operator<<(std::ostream& os, const arp_ods::orders::mode& mode)
{
    switch (mode)
    {
        case MODE_INIT:
            return os << "MODE_INIT";
            break;
        case MODE_RUN:
            return os << "MODE_RUN";
            break;
        case MODE_APPROACH:
            return os << "MODE_APPROACH";
            break;
        case MODE_DONE:
            return os << "MODE_DONE";
            break;
        case MODE_ERROR:
            return os << "MODE_ERROR";
            break;
        case MODE_PASS:
            return os << "MODE_PASS";
            break;
        default:
            return os << "MODE_UNKNOWN";
            break;
    }
    return os << "MODE_UNKNOWN";
}


ModeSelector::ModeSelector()
{
    m_beginMotionState = UbiquityMotionState();
    m_endMotionState = UbiquityMotionState();
    m_pass = false;
    m_currentMode = MODE_INIT;
    m_passTime = 0;
    // -1 is used to recognize non initialized time
    m_initTime = -1;
    m_approachTime = -1;

    //conf
    m_conf.ANGLE_ACCURACY = -1.0;
    m_conf.DISTANCE_ACCURACY = -1.0;
    m_conf.RADIUS_APPROACH_ZONE = -1.0;
    m_conf.DISTANCE_ACCURACY = -1.0;
    m_conf.ANGLE_ACCURACY = -1.0;
    m_conf.PASS_TIMEOUT = -1.0;
    m_conf.ORDER_TIMEOUT = -1.0;
    m_conf.VEL_FINAL = -1.0;
    m_conf.LIN_VEL_MAX = -1.0;
    m_conf.ANG_VEL_MAX = -1.0;
    m_conf.LIN_DEC = -1.0;
    m_conf.ANG_DEC = -1.0;

}

void ModeSelector::resetMode()
{
    m_currentMode = MODE_INIT;
}

void ModeSelector::setConf(config conf)
{
    m_conf=conf;
}

void ModeSelector::switchInit(UbiquityMotionState currentMotionState)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();
    testTimeout();
    m_currentMode = MODE_RUN;
    m_runTime = getTime();

    Log(INFO) << "[" << m_endMotionState.toString()<< "] entered MODE_INIT at time " << m_initTime;
}

void ModeSelector::switchRun(UbiquityMotionState currentMotionState)
{
    if (getRemainingDistance(currentMotionState) <= m_conf.RADIUS_APPROACH_ZONE)
    {
        if (getPass())
        {
            Log(INFO) << "switched MODE_RUN --> MODE_PASS";
            m_currentMode = MODE_PASS;
            m_passTime = getTime();
            return;
        }
        else
        {
            Log(INFO) << "switched MODE_RUN --> MODE_APPROACH";
            m_currentMode = MODE_APPROACH;
            m_approachTime= getTime();
            return;
        }
    }
    testTimeout();
}

void ModeSelector::switchApproach(UbiquityMotionState currentMotionState)
{/*
    double distance_error = getRemainingDistance(currentMotionState);
    double angle_error = getRemainingAngle(currentMotionState);

    if (distance_error < m_conf.DISTANCE_ACCURACY && fabs(angle_error) < m_conf.ANGLE_ACCURACY)
    {
        Log(INFO) << "switched MODE_APPROACH --> MODE_DONE";
        char string [250];
        sprintf(string,"(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentMotionState.getPosition().x(), currentMotionState.getPosition().y(),
                        rad2deg(currentMotionState.getPosition().h()), distance_error * 1000, rad2deg(angle_error));
        Log(INFO) << string;
        m_currentMode = MODE_DONE;
        return;
    }*/
    testTimeout();

}

void ModeSelector::switchDone(UbiquityMotionState currentMotionState)
{

}

void ModeSelector::switchError(UbiquityMotionState currentMotionState)
{

}

void ModeSelector::switchPass(UbiquityMotionState currentMotionState)
{/*
    double t = getTime();
    double dt = t - m_passTime;
    if (dt < 0 || dt > m_conf.PASS_TIMEOUT)
    {
        Log(INFO) << "switched MODE_PASS --> MODE_DONE because of dt= "<< dt;
        m_currentMode = MODE_DONE;
        return;
    }*/

    testTimeout();
}

void ModeSelector::switchMode(UbiquityMotionState currentMotionState)
{
    switch (m_currentMode)
    {
        case MODE_INIT:
            switchInit(currentMotionState);
            break;
        case MODE_RUN:
            switchRun(currentMotionState);
            break;
        case MODE_APPROACH:
            switchApproach(currentMotionState);
            break;
        case MODE_DONE:
            switchDone(currentMotionState);
            break;
        case MODE_ERROR:
            switchError(currentMotionState);
            break;
        case MODE_PASS:
            switchPass(currentMotionState);
            break;
        default:
            break;
    }
}

double ModeSelector::getRemainingDistance(UbiquityMotionState currentMotionState)
{
    return currentMotionState.getPosition().distanceTo(m_endMotionState.getPosition());
}

double ModeSelector::getRemainingAngle(UbiquityMotionState currentMotionState)
{
    double e_theta = currentMotionState.getPosition().angleTo(m_endMotionState.getPosition());
    return e_theta;
}

double ModeSelector::getCoveredDistance(UbiquityMotionState currentMotionState)
{
    return currentMotionState.getPosition().distanceTo(m_beginMotionState.getPosition());
}

UbiquityMotionState ModeSelector::getBeginMotionState() const
{
    return m_beginMotionState;
}

UbiquityMotionState ModeSelector::getEndMotionState() const
{
    return m_endMotionState;
}

Pose2D ModeSelector::getCpoint() const
{
    return m_cpoint;
}

bool ModeSelector::getPass() const
{
    return m_pass;
}

mode ModeSelector::getMode() const
{
    return m_currentMode;
}

void ModeSelector::setPass(bool pass)
{
    m_pass = pass;
}

void ModeSelector::setPassSpeed(double passSpeed)
{
    if( passSpeed <= 0.0 )
    {
        m_passSpeed = 0.1;
        Log(ERROR) << "ModeSelector::setPassSpeed(" << passSpeed << ") : Pass Speed is incorrect, should be positive";
    }
    else
        m_passSpeed = passSpeed;
}

void ModeSelector::setBeginMotionState(UbiquityMotionState beginMotionState)
{
    beginMotionState.getPosition().h(betweenMinusPiAndPlusPi(beginMotionState.getPosition().h()));
    this->m_beginMotionState = beginMotionState;
}

void ModeSelector::setEndMotionState(UbiquityMotionState endMotionState)
{
    endMotionState.getPosition().h(betweenMinusPiAndPlusPi(endMotionState.getPosition().h()));
    this->m_endMotionState = endMotionState;
}


void ModeSelector::setCpoint(Pose2D cpoint)
{
    cpoint.h(betweenMinusPiAndPlusPi(cpoint.h()));
    this->m_cpoint = cpoint;
}

void ModeSelector::testTimeout()
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

