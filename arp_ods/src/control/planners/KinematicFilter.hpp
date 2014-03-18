/*
 * KinematicFilter.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#ifndef KINEMATICFILTER_HPP_
#define KINEMATICFILTER_HPP_

#include <math/core>
#include <models/core>

namespace arp_ods
{

class KinematicFilter
{
    public:
        KinematicFilter();
        virtual ~KinematicFilter();

        /**
         * Check if the current motor state command is reachable
         * @param desiredState : the stat we try to reach already containing the solution for angle.
         * @param measuredState : the current motor state.
         * @param params : geometrical parameters required to use UbiquityKinematics models
         * @return a percentage between 0 and 1 which give a rates to apply on the desiredTwist commanded
         */
        static bool isMotorStateReachable(const arp_model::MotorState & desiredMS,
                                        const arp_model::MotorState & measuredMS,
                                        const arp_model::UbiquityParams & params,
                                        const double & dt);
};

}
#endif /* KINEMATICFILTER_HPP_ */
