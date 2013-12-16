/*
 * ReplayOrder.hpp
 *
 *  Created on: 210 april 2012
 *      Author: RMO
 */

#ifndef STAYINPOSITION_HPP_
#define STAYINPOSITION_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>

using namespace boost;

namespace arp_ods{ namespace orders
{

class StayOrder: public MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StayOrder(const OrderGoalConstPtr &goal,UbiquityMotionState currentMotionState, UbiquityParams params );

        /**
         *
         */
        virtual arp_math::ICRSpeed computeSpeed(UbiquityMotionState currentMotionState, double dt);


};

}}

#endif /* STAYINPOSITION_HPP_ */
