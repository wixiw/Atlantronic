/*
 * RosOdsItf.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ROSODSITF_HPP_
#define ROSODSITF_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>
#include <arp_core/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include "control/orders/orders.h"
#include <arp_ods/OrderAction.h>
#include <arp_ods/SetVMax.h>

namespace arp_ods
{

class RosOdsItf: public OdsTaskContext
{
    public:
        RosOdsItf(std::string const name);

    protected:
        InputPort<arp_math::EstimatedPose2D> inPose;
        InputPort<bool> inCurrentOrderIsFinished;
        InputPort<bool> inCurrentOrderIsInError;
        OutputPort< shared_ptr<MotionOrder> > outOrder;

        /**
         * Actionlib server.
         * Simple version is perfect here.
         */
        actionlib::SimpleActionServer<arp_ods::OrderAction> m_actionServer;

        /** The current order to do. There is always one at a time since
         * newOrderCB is blocking by design of the action lib
         */
        shared_ptr<MotionOrder> m_order;

        /** Motion control configuration. They are feeded with rosparam during init */
        order::config m_orderConfig;

        /**
         * Tells if the current order is finished with success. It is finished when the robot is in PASS mode when the order
         * is a "pass" type or When the mode is DONE.
         */
        bool isOrderFinished();

        /**
         * Called when a new order is received
         * This *MUST* be a blocking function since ActionLib wants it.
         * The caller is not blocked because a background thread is waiting
         * the action result to call a callback.
         * In a way this should be done independantly of the periodic job
         * to publish velocity commands. For simplicity, this callback stay here
         * but it is as if it was independant
         */
        void newOrderCB(const OrderGoalConstPtr &goal);

        /**
         * Read ROS parameters from the ROS server
         */
        void updateParams();

        /**
         * called when setvmax service is called
         */
        bool setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res);
};

} /* namespace arp_rlu */
#endif /* ROSODSITF_HPP_ */