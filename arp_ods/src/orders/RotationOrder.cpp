/*
 * RotationOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "RotationOrder.hpp"

namespace arp_ods
{

RotationOrder::RotationOrder()
{
    m_type = ROTATE;
    m_reverse = false;
    m_pass = false;
}

void RotationOrder::switchInit(arp_core::Pose currentPosition)
{
    m_beginPose = currentPosition;
    m_endPose.x = m_beginPose.x;
    m_endPose.y = m_endPose.y;
    m_reverse = false;
    m_pass = false;

    ROS_INFO("switched to mode MODE_APPROACH from MODE_INIT because it's a RotationOrder");
    m_currentMode = MODE_APPROACH;
    return;
}

void RotationOrder::setReverse(bool reverse)
{
    m_reverse = false;
}

void RotationOrder::setPass(bool pass)
{
    m_pass = false;
}

}
