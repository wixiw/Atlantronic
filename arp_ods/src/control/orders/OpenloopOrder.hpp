/*
 * OmnidirectOrder.hpp
 *
 *  Created on: 210 april 2012
 *      Author: RMO
 */

#ifndef OPENLOOPORDER_HPP_
#define OPENLOOPORDER_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>
#include <models/core>

using namespace boost;

namespace arp_ods{ namespace orders
{

class OpenloopOrder: public MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OpenloopOrder(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
UbiquityParams params);


        /**
         *
         */
        virtual ICRSpeed computeSpeed(UbiquityMotionState currentMotionState, double dt);

        /*
         * returns the error on position between actual an objective,  in table referential
         */
        arp_math::Pose2D getPositionError(arp_math::Pose2D currentPosition);

    protected:

        /*
         * twist of precedent turn
         */
        arp_math::Twist2D m_v_correction_old;

        //surcharger pour supprimer le mode approche en mode normal. En mode PASS la distance approche est utilisée
        void switchRun(UbiquityMotionState currentMotionState);


        static const double MAX_OPENLOOP_TIME=5.0;
};

}}

#endif /* OPENLOOPORDER_HPP_ */
