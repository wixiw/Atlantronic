/*
 * OnlineTrajectoryGenerator.hpp
 *      Sept 25, 2012
 *      Author: Romain Moulin
 */

#ifndef ONLINETRAJECTORYGENERATOR_HPP_
#define ONLINETRAJECTORYGENERATOR_HPP_

#include <math/core>
#include <models/core>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include "PosVelAcc.hpp"

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace std;

/****************
 * This is a wrapper for the calls to Reflexxes
 *
 * Reflexxes is used only in 1-axis
 * with Type IV library, it means computing SCurves, for jerk limitation
 *
 * Reflexxes is able to compute the time-optimal trajectory for any (position, velocity, acceleration) to a target (position, velocity, 0)
 * while ensuring the limits of speed, acceleration and jerk
 */

namespace arp_ods
{
    class OnlineTrajectoryGenerator
    {
        public:
            OnlineTrajectoryGenerator();
            ~OnlineTrajectoryGenerator();

            ReflexxesAPI *RML ;
            bool computeNextStep(const PosVelAcc & iStart, const PosVelAcc & iEnd, const double & iMaxVelocity,const double & iMaxAcceleration,const double & iMaxJerk,  PosVelAcc & oNext);
            bool computeNextStepCheap(const PosVelAcc & iStart, const PosVelAcc & iEnd, const double & iMaxVelocity,const double & iMaxAcceleration,const double & iMaxJerk,  PosVelAcc & oNext);

            RMLPositionInputParameters  *IP                        ;

            RMLPositionOutputParameters *OP                         ;

            RMLPositionFlags            Flags                                   ;

            double m_dt;
    };


}
#endif /* ONLINETRAJECTORYGENERATOR_HPP_ */
