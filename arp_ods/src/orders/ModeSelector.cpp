/*
 * ModeSelector.cpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */

#include "math/math.hpp"
#include "ModeSelector.hpp"

using namespace arp_core;
using namespace arp_math;

// Displaying:
std::ostream& operator<<(std::ostream& os, const arp_ods::mode& mode)
{
    switch (mode)
    {
        case arp_ods::MODE_INIT:
            return os << "MODE_INIT";
            break;
        case arp_ods::MODE_RUN:
            return os << "MODE_RUN";
            break;
        case arp_ods::MODE_APPROACH:
            return os << "MODE_APPROACH";
            break;
        case arp_ods::MODE_DONE:
            return os << "MODE_DONE";
            break;
        case arp_ods::MODE_ERROR:
            return os << "MODE_ERROR";
            break;
        case arp_ods::MODE_PASS:
            return os << "MODE_PASS";
            break;
        default:
            return os << "MODE_UNKNOWN";
            break;
    }
}

namespace arp_ods
{

ModeSelector::ModeSelector()
{
    m_beginPose = Pose();
    m_endPose = Pose();
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

void ModeSelector::setDefaults(order::config conf)
{
    m_angleAccuracy = conf.ANGLE_ACCURACY;
    m_distanceAccuracy = conf.DISTANCE_ACCURACY;
    m_radiusInitZone = conf.RADIUS_INIT_ZONE;
    m_radiusApproachZone = conf.RADIUS_APPROACH_ZONE;
    m_passTimeout = conf.PASS_TIMEOUT;
    m_orderTimeout = conf.ORDER_TIMEOUT;
}

void ModeSelector::switchInit(arp_core::Pose currentPosition)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = getTime();

    if (getCoveredDistance(currentPosition) >= getRadiusInitZone())
    {
        ROS_INFO("switched MODE_INIT --> MODE_RUN ");
        m_currentMode = MODE_RUN;
        return;
    }
    testTimeout("MODE_INIT");
}

void ModeSelector::switchRun(arp_core::Pose currentPosition)
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
    testTimeout("MODE_RUN");
}

void ModeSelector::switchApproach(arp_core::Pose currentPosition)
{
    double distance_error = getRemainingDistance(currentPosition);
    double angle_error = getRemainingAngle(currentPosition);

    if (distance_error < m_distanceAccuracy && fabs(angle_error) < m_angleAccuracy)
    {
        ROS_INFO("switched MODE_APPROACH --> MODE_DONE");
        ROS_INFO("(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentPosition.x, currentPosition.y,
                rad2deg(currentPosition.theta), distance_error * 1000, rad2deg(angle_error));
        m_currentMode = MODE_DONE;
        return;
    }
    testTimeout("MODE_APPROACH");

}

void ModeSelector::switchDone(arp_core::Pose currentPosition)
{

}

void ModeSelector::switchError(arp_core::Pose currentPosition)
{

}

void ModeSelector::switchPass(arp_core::Pose currentPosition)
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

void ModeSelector::switchMode(arp_core::Pose currentPosition)
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

double ModeSelector::getRemainingDistance(arp_core::Pose currentPosition)
{
    return distance(currentPosition, m_endPose);
}

double ModeSelector::getRemainingAngle(arp_core::Pose currentPosition)
{
    double e_theta = angle(currentPosition, m_endPose);
    //ROS_WARN("e_theta %0.3f",e_theta);
    return e_theta;
}

double ModeSelector::getCoveredDistance(arp_core::Pose currentPosition)
{
    return distance(m_beginPose, currentPosition);
}

arp_core::Pose ModeSelector::getBeginPose() const
{
    return m_beginPose;
}

arp_core::Pose ModeSelector::getEndPose() const
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

double ModeSelector::getRadiusInitZone() const
{
    return m_radiusInitZone;
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

void ModeSelector::setRadiusInitZone(double m_radiusInitZone)
{
    this->m_radiusInitZone = m_radiusInitZone;
}

void ModeSelector::setAngleAccuracy(double m_angleAccuracy)
{
    this->m_angleAccuracy = m_angleAccuracy;
}

void ModeSelector::setDistanceAccurancy(double m_distanceAccurancy)
{
    this->m_distanceAccuracy = m_distanceAccurancy;
}

void ModeSelector::setBeginPose(arp_core::Pose beginPose)
{
    beginPose.theta = normalizeAngle(beginPose.theta);
    this->m_beginPose = beginPose;
}

void ModeSelector::setEndPose(arp_core::Pose endPose)
{
    endPose.theta = normalizeAngle(endPose.theta);
    this->m_endPose = endPose;
}

double ModeSelector::distance(arp_core::Pose a, arp_core::Pose b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

double ModeSelector::angle(arp_core::Pose a, arp_core::Pose b)
{
    return normalizeAngle(b.theta - a.theta);
}

void ModeSelector::testTimeout(std::string from_mode)
{
    double t = getTime();
    double dt = t - m_initTime;

    if (m_initTime != -1 and dt > m_orderTimeout)
    {
        ROS_INFO("switched %s --> MODE_ERROR because of dt=%0.3f", from_mode.c_str(), dt);
        m_currentMode = MODE_ERROR;
        return;
    }
}

}
