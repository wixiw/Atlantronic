/*
 * UbiquityMotionState.cpp
 *
 *  Created on: Nov 30, 2013
 *      Author: ard
 */

#include "UbiquityMotionState.hpp"

using namespace arp_math;

UbiquityMotionState::UbiquityMotionState() :
        position(), speed()
{
}

UbiquityMotionState::UbiquityMotionState(EstimatedPose2D position, EstimatedICRSpeed speed)
{
    setPosition(position);
    setSpeed(speed);
}

std::string UbiquityMotionState::toString() const
{
    std::stringstream ss;
    ss << std::endl;
    ss << "MotionState       ";
    ss << "currentPosition : " << getPosition().toString();
    ss << "currentSpeed :    " << getSpeed().toString();
    ss << std::endl;
    return ss.str();
}

EstimatedPose2D UbiquityMotionState::getPosition() const
{
    return position;
}

EstimatedICRSpeed UbiquityMotionState::getSpeed() const
{
    return speed;
}

void UbiquityMotionState::setPosition(EstimatedPose2D position)
{
    this->position = position;
}

void UbiquityMotionState::setSpeed(EstimatedICRSpeed speed)
{
    this->speed = speed;
}
