/*
 * OrderFactory.h
 *
 *  Created on: Dec 15, 2013
 *      Author: ard
 */

#ifndef ORDERFACTORY_H_
#define ORDERFACTORY_H_

#include "control/orders/orders.h"
#include <models/core>
#include <math/core>
#include <boost/shared_ptr.hpp>
#include <arp_ods/OrderAction.h>

using namespace arp_math;

namespace arp_ods
{
namespace orders
{


class OrderFactory
{
    public:


        /**
         * Factory to create an order
         * @param goal : action lib goal to process
         * @param currentMotionState : current motion state of the robot
         * @param conf : automation parameters (gains)
         * @return : a MotionOrder to execute
         */
        static boost::shared_ptr<MotionOrder> createOrder(const OrderGoalConstPtr &goal,
                UbiquityMotionState currentMotionState, UbiquityParams params);

        static boost::shared_ptr<MotionOrder> createStayOrder(UbiquityMotionState currentMotionState, UbiquityParams params);

        static boost::shared_ptr<MotionOrder> createDefaultOrder();
};

}}
#endif /* ORDERFACTORY_H_ */
