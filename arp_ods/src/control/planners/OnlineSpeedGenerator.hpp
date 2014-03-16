/*
 * OnlineSpeedGenerator.hpp
 *
 *  Created on: Mar 15, 2014
 *      Author: ard
 */

#ifndef ONLINESPEEDGENERATOR_HPP_
#define ONLINESPEEDGENERATOR_HPP_

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include "PosVelAcc.hpp"

namespace arp_ods
{

class OnlineSpeedGenerator
{
    public:
        /** This dt is the expected period (in s) for the next step, so it can be the theorical value*/
        OnlineSpeedGenerator(double theoricalPeriod);
        virtual ~OnlineSpeedGenerator();

        /** Define a new maximal speed in m/s */
        void setMaxSpeed(double vMax);

        /** Define a new maximal acc in m/s2 */
        void setMaxAcc(double accMax);

        /** Define a new maximal jerk in m/s3 */
        void setMaxJerk(double jerkMax);

        /** Define new Dynamic limitations */
        void setDynamicLimitations(double vMax, double accMax, double jerkMax);

        /** Compute the next speed to apply toward targetSpeed ensuring dynamic limitations are fitted. Returns false on failure*/
        bool computeNextStep(double targetSpeed, PosVelAcc currentState, PosVelAcc& reachableState);

    protected:
        static const int DOF = 1;
        ReflexxesAPI m_RML;
        RMLVelocityFlags m_flags;

        double m_maxSpeed;
        double m_maxAcc;
        double m_maxJerk;
};

} /* namespace arp_ods */
#endif /* ONLINESPEEDGENERATOR_HPP_ */
