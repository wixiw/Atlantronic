/*
 * RotationOrder.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef ROTATIONORDER_HPP_
#define ROTATIONORDER_HPP_

#include "FantomOrder.hpp"
#include <boost/shared_ptr.hpp>

using namespace boost;

namespace arp_ods{ namespace orders
{

/**
 * This order allows to do a pure rotation
 */
class RotationOrder: public arp_ods::orders::FantomOrder
{
    public:
        RotationOrder();
        RotationOrder(MotionOrder order);

        /** Override to define specific parameters*/
        static shared_ptr<MotionOrder>  createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, orders::config conf );

        /** Override to go in approach mode directly */
        virtual void switchInit(arp_math::Pose2D currentPosition);

        /** Override to forbids another value than false */
        void setReverse(bool reverse);

        /** Override to forbids another value than false */
        void setPass(bool pass);
};

}}

#endif /* ROTATIONORDER_HPP_ */
