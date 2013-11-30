/*
 * UbiquityMotionState.hpp
 *
 *  Created on: Nov 30, 2013
 *      Author: ard
 */

#ifndef UBIQUITYMOTIONSTATE_HPP_
#define UBIQUITYMOTIONSTATE_HPP_

/**
 * This file is separated from UbiquityStates because of orogen limitations
 */
#include <math/core>

namespace arp_math
{

/** this class represents position, speed (and acceleration potentially) in "ICRSpeed" language, and on robot referential*/

class UbiquityMotionState
{
    public:
        EstimatedPose2D position;
        EstimatedICRSpeed speed;

        UbiquityMotionState();
        UbiquityMotionState(EstimatedPose2D position, EstimatedICRSpeed speed);

        std::string toString() const;

        EstimatedPose2D getPosition() const;
        EstimatedICRSpeed getSpeed() const;
        void setPosition(EstimatedPose2D position);
        void setSpeed(EstimatedICRSpeed speed);
};

} /* namespace arp_math */
#endif /* UBIQUITYMOTIONSTATE_HPP_ */
