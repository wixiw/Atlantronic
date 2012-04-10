/*
 * OmnidirectOrder.hpp
 *
 *  Created on: 210 april 2012
 *      Author: RMO
 */

#ifndef OMNIDIRECTORDER_HPP_
#define OMNIDIRECTORDER_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>

using namespace boost;

namespace arp_ods
{

class OmnidirectOrder: public arp_ods::MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OmnidirectOrder();
        OmnidirectOrder(MotionOrder);

        /**
         * Define configurable attributes to their default values
         */
        virtual void setDefaults(order::config conf);

        /**
         * Override to define specific parameters
         */
        static shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, order::config conf  );

        /**
         *
         */
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition, double dt);

    protected:
        double DECLIN;
        double DECROT;
        double VMAXLIN;
        double VMAXROT;



};

}

#endif /* OMNIDIRECTORDER_HPP_ */
