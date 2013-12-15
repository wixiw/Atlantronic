/*
 * OrderFactory.cpp
 *
 *  Created on: Dec 15, 2013
 *      Author: ard
 */

#include "OrderFactory.hpp"
#include "control/orders/orders.h"

using namespace arp_math;

namespace arp_ods
{
namespace orders
{



boost::shared_ptr<MotionOrder> OrderFactory::createOrder(const OrderGoalConstPtr &goal, UbiquityMotionState currentMotionState,
        orders::config conf)
{
    if (goal->move_type == "OMNIDIRECT2")
        {
        shared_ptr<OmnidirectOrder2> orderOmni(new OmnidirectOrder2(goal, currentMotionState, conf));
        return static_cast<shared_ptr<MotionOrder> >(orderOmni);
        }
    else if (goal->move_type == "OPENLOOP")
        {
        shared_ptr<OpenloopOrder> orderOpenLoop(new OpenloopOrder(goal, currentMotionState, conf));
        return static_cast<shared_ptr<MotionOrder> >(orderOpenLoop);
        }
    else if (goal->move_type == "REPLAY")
        {
        shared_ptr<ReplayOrder> orderReplay(new ReplayOrder(goal, currentMotionState, conf));
        return static_cast<shared_ptr<MotionOrder> >(orderReplay);
        }
    else if (goal->move_type == "STAY")
        {
        shared_ptr<StayOrder> orderStay(new StayOrder(goal, currentMotionState, conf));
        return static_cast<shared_ptr<MotionOrder> >(orderStay);
        }

   // in case move type was not known
   return OrderFactory::createDefaultOrder();

}

boost::shared_ptr<MotionOrder> OrderFactory::createStayOrder(UbiquityMotionState currentMotionState, orders::config conf)
{
    shared_ptr<StayOrder> order(new StayOrder(OrderGoalConstPtr(), currentMotionState, conf));
    return static_cast<shared_ptr<MotionOrder> >(order);
}

boost::shared_ptr<MotionOrder> OrderFactory::createDefaultOrder()
{
    shared_ptr<StayOrder> order(new StayOrder(OrderGoalConstPtr(),UbiquityMotionState(),orders::config()));
    return static_cast<shared_ptr<MotionOrder> >(order);
}

}
}
