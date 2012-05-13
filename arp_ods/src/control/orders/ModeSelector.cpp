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
}


ModeSelector::ModeSelector()
{
    m_beginPose = arp_math::Pose2D();
    m_endPose = arp_math::Pose2D();
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

void ModeSelector::switchInit(arp_math::Pose2D currentPosition)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();
    testTimeout();
    m_currentMode = MODE_RUN;
    m_runTime = getTime();

    Log(INFO) << "[" << m_endPose.toString()<< "] entered MODE_INIT at time " << m_initTime;
}

void ModeSelector::switchRun(arp_math::Pose2D currentPosition)
{
    if (getRemainingDistance(currentPosition) <= m_conf.RADIUS_APPROACH_ZONE)
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

void ModeSelector::switchApproach(arp_math::Pose2D currentPosition)
{
    double distance_error = getRemainingDistance(currentPosition);
    double angle_error = getRemainingAngle(currentPosition);

    if (distance_error < m_conf.DISTANCE_ACCURACY && fabs(angle_error) < m_conf.ANGLE_ACCURACY)
    {
        Log(INFO) << "switched MODE_APPROACH --> MODE_DONE";
        char string [250];
        sprintf(string,"(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentPosition.x(), currentPosition.y(),
                        rad2deg(currentPosition.h()), distance_error * 1000, rad2deg(angle_error));
        Log(INFO) << string;
        m_currentMode = MODE_DONE;
        return;
    }
    testTimeout();

}

void ModeSelector::switchDone(arp_math::Pose2D currentPosition)
{

}

void ModeSelector::switchError(arp_math::Pose2D currentPosition)
{

}

void ModeSelector::switchPass(arp_math::Pose2D currentPosition)
{
    double t = getTime();
    double dt = t - m_passTime;
    if (dt < 0 || dt > m_conf.PASS_TIMEOUT)
    {
        Log(INFO) << "switched MODE_PASS --> MODE_DONE because of dt= "<< dt;
        m_currentMode = MODE_DONE;
        return;
    }
}

void ModeSelector::switchMode(Pose2D currentPosition)
{
    switch (m_currentMode)
    {
        case MODE_INIT:
            switchInit(currentPosition);
            break;
        case MODE_RUN:
            switchRun(currentPosition);
            break;
        case MODE_APPROACH:
            switchApproach(currentPosition);
            break;
        case MODE_DONE:
            switchDone(currentPosition);
            break;
        case MODE_ERROR:
            switchError(currentPosition);
            break;
        case MODE_PASS:
            switchPass(currentPosition);
            break;
        default:
            break;
    }
}

double ModeSelector::getRemainingDistance(arp_math::Pose2D currentPosition)
{
    return currentPosition.distanceTo(m_endPose);
}

double ModeSelector::getRemainingAngle(arp_math::Pose2D currentPosition)
{
    double e_theta = currentPosition.angleTo(m_endPose);
    return e_theta;
}

double ModeSelector::getCoveredDistance(arp_math::Pose2D currentPosition)
{
    return currentPosition.distanceTo(m_beginPose);
}

arp_math::Pose2D ModeSelector::getBeginPose() const
{
    return m_beginPose;
}

arp_math::Pose2D ModeSelector::getEndPose() const
{
    return m_endPose;
}

arp_math::Pose2D ModeSelector::getCpoint() const
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

void ModeSelector::setBeginPose(Pose2D beginPose)
{
    beginPose.h(betweenMinusPiAndPlusPi(beginPose.h()));
    this->m_beginPose = beginPose;
}

void ModeSelector::setEndPose(Pose2D endPose)
{
    endPose.h(betweenMinusPiAndPlusPi(endPose.h()));
    this->m_endPose = endPose;
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

