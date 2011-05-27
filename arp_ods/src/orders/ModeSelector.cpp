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
    setDefaults();
}

void ModeSelector::resetMode()
{
    m_currentMode = MODE_INIT;
}

void ModeSelector::setDefaults()
{
    //definition de valeurs par defaut
    m_angleAccuracy = 0.2;
    m_distanceAccurancy = 0.020;
    m_radiusApproachZone = 0.050;
    m_radiusInitZone = 0.0;
    m_passTimeout = 0.5;
}

void ModeSelector::switchInit(arp_core::Pose currentPosition)
{
    if (getCoveredDistance(currentPosition) >= getRadiusInitZone())
    {
        ROS_INFO("switched to mode MODE_RUN from MODE_INIT");
        m_currentMode = MODE_RUN;
        return;
    }
}

void ModeSelector::switchRun(arp_core::Pose currentPosition)
{
    if (getRemainingDistance(currentPosition) <= getRadiusApproachZone())
    {
        if( getPass() )
        {
            ROS_INFO("switched to mode MODE_PASS from MODE_RUN");
            m_currentMode = MODE_PASS;
            timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            m_passTime = now.tv_sec + (double)(now.tv_nsec)/1E9;
            return;
        }
        else
        {
            ROS_INFO("switched to mode MODE_APPROACH from MODE_RUN");
            m_currentMode = MODE_APPROACH;
            return;
        }
    }
}

void ModeSelector::switchApproach(arp_core::Pose currentPosition)
{
    double distance_error = getRemainingDistance(currentPosition);
    double angle_error = getRemainingAngle(currentPosition);

    if (distance_error < m_distanceAccurancy && fabs(angle_error) < m_angleAccuracy)
    {

        ROS_INFO("switched to mode MODE_DONE from MODE_APPROACH :");
        ROS_INFO("(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentPosition.x, currentPosition.y,
                rad2deg(currentPosition.theta), distance_error * 1000, rad2deg(angle_error));
        m_currentMode = MODE_DONE;
        return;
    }

}

void ModeSelector::switchDone(arp_core::Pose currentPosition)
{

}

void ModeSelector::switchError(arp_core::Pose currentPosition)
{

}

void ModeSelector::switchPass(arp_core::Pose currentPosition)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double t = now.tv_sec + (double)(now.tv_nsec)/1E9;
    double dt = t - m_passTime;
    if( dt < 0 || dt > m_passTimeout )
    {
        ROS_INFO("switched to mode MODE_DONE from MODE_PASS because of dt=%0.3f",dt);
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
    return angle(currentPosition, m_endPose);
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
    return m_distanceAccurancy;
}

void ModeSelector::setPass(bool pass)
{
    m_pass = pass;
}

void ModeSelector::setPassTimeout(double timeout)
{
    m_passTimeout = timeout;
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
    this->m_distanceAccurancy = m_distanceAccurancy;
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

}
