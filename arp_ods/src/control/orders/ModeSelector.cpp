/*
 * ModeSelector.cpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */
#include "ModeSelector.hpp"

#include <math/core>
#include <ros/ros.h>

using namespace arp_math;
using namespace arp_ods;
using namespace orders;

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
    m_angleAccuracy = -1;
    m_distanceAccuracy = -1;
    m_radiusInitZone = -1;
    m_radiusApproachZone = -1;
    m_passTimeout = -1;
    m_orderTimeout = -1;
}

void ModeSelector::resetMode()
{
    m_currentMode = MODE_INIT;
}

void ModeSelector::setDefaults(config conf)
{
    m_angleAccuracy = conf.ANGLE_ACCURACY;
    m_distanceAccuracy = conf.DISTANCE_ACCURACY;
    m_radiusApproachZone = conf.RADIUS_APPROACH_ZONE;
    m_passTimeout = conf.PASS_TIMEOUT;
    m_orderTimeout = conf.ORDER_TIMEOUT;
}

void ModeSelector::switchInit(arp_math::Pose2D currentPosition)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();
    testTimeout();
    m_currentMode = MODE_RUN;
}

void ModeSelector::switchRun(arp_math::Pose2D currentPosition)
{
    if (getRemainingDistance(currentPosition) <= getRadiusApproachZone())
    {
        if (getPass())
        {
            ROS_INFO("switched MODE_RUN --> MODE_PASS");
            m_currentMode = MODE_PASS;
            m_passTime = getTime();
            return;
        }
        else
        {
            ROS_INFO("switched MODE_RUN --> MODE_APPROACH");
            m_currentMode = MODE_APPROACH;
            return;
        }
    }
    testTimeout();
}

void ModeSelector::switchApproach(arp_math::Pose2D currentPosition)
{
    double distance_error = getRemainingDistance(currentPosition);
    double angle_error = getRemainingAngle(currentPosition);

    if (distance_error < m_distanceAccuracy && fabs(angle_error) < m_angleAccuracy)
    {
        ROS_INFO("switched MODE_APPROACH --> MODE_DONE");
        ROS_INFO("(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentPosition.x(), currentPosition.y(),
                rad2deg(currentPosition.h()), distance_error * 1000, rad2deg(angle_error));
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
    if (dt < 0 || dt > m_passTimeout)
    {
        ROS_INFO("switched MODE_PASS --> MODE_DONE because of dt=%0.3f", dt);
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
    //ROS_WARN("e_theta %0.3f",e_theta);
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

bool ModeSelector::getPass() const
{
    return m_pass;
}

mode ModeSelector::getMode() const
{
    return m_currentMode;
}

double ModeSelector::getRadiusApproachZone() const
{
    return m_radiusApproachZone;
}

double ModeSelector::getAngleAccuracy() const
{
    return m_angleAccuracy;
}

double ModeSelector::getDistanceAccurancy() const
{
    return m_distanceAccuracy;
}

void ModeSelector::setPass(bool pass)
{
    m_pass = pass;
}

void ModeSelector::setPassTimeout(double timeout)
{
    m_passTimeout = timeout;
}

void ModeSelector::setOrderTimeout(double timeout)
{
    m_orderTimeout = timeout;
}

void ModeSelector::setRadiusApproachZone(double m_radiusApproachZone)
{
    this->m_radiusApproachZone = m_radiusApproachZone;
}

void ModeSelector::setAngleAccuracy(double m_angleAccuracy)
{
    this->m_angleAccuracy = m_angleAccuracy;
}

void ModeSelector::setDistanceAccurancy(double m_distanceAccurancy)
{
    this->m_distanceAccuracy = m_distanceAccurancy;
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

void ModeSelector::testTimeout()
{
    double t = getTime();
    double dt = t - m_initTime;

    if (m_initTime != -1 and dt > 600) //dt > m_orderTimeout
    {
        ROS_INFO_STREAM("switched from " << getMode() << " to MODE_ERROR because of dt=" << dt);
        m_currentMode = MODE_ERROR;
        return;
    }
}

