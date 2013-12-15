/*
 * ReplayOrder.hpp
 *
 *  Created on: 210 april 2012
 *      Author: RMO
 */

#ifndef REPLAYORDER_HPP_
#define REPLAYORDER_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>

using namespace boost;

namespace arp_ods{ namespace orders
{

class ReplayOrder: public MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReplayOrder(const OrderGoalConstPtr &goal,UbiquityMotionState currentMotionState, orders::config conf );

        /**
         *
         */
        virtual arp_math::ICRSpeed computeSpeed(UbiquityMotionState currentMotionState,UbiquityParams params, double dt);
        /*
         * returns the error on position between actual an objective,  in table referential
         */
        arp_math::Pose2D getPositionError(arp_math::Pose2D currentPosition);

    protected:


        //surcharger pour supprimer le mode approche en mode normal. En mode PASS la distance approche est utilisée
        void switchRun(arp_math::Pose2D currentPosition);

        /** replay duraction and current time */
        double m_replayDuration;
        double m_replayTime;


};

}}

#endif /* REPLAYORDER_HPP_ */
