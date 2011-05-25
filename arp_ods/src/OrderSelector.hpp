/*
 * OrderSelector.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef ORDERSELECTOR_HPP_
#define ORDERSELECTOR_HPP_

#include <boost/shared_ptr.hpp>
#include "orders/orders.h"

using namespace boost;

namespace arp_ods
{

enum MotionState
{
    IDLE,
    WORK,
    HALT
};

/**
 * This class is responsible for the order's linking.
 * It has 3 states IDLE->WORK->HALT. A new order is processed
 * when the Robot is in IDLE state or when an order is finished (MODE_DONE)
 * When there is no following order, or when the interrupt function is called
 * the motion is stopped in the HALT state.
 * The state became IDLE as soon as the robot has a null speed from HALT.
 *
 * You have to call the switchState function periodically for it to work;
 *
 * you are NOT supposed to accept a new order if the requested order is different of StayInPosition
 */

class OrderSelector
{
    public:
        OrderSelector();

        /**
         * Call this periodically to update the state
         */
         void switchState(arp_core::Pose currentPose);

         /**
          * Interrupt the current order
          */
         void interrupt();

         /**
          * Returns the current order
          * It never returns a NULL pointer, you don't have to check
          */
         shared_ptr<MotionOrder> getCurrentOrder();

         /**
          * Returns the requested order
          * It is leaved to StayInPosition when no order is present
          */
         shared_ptr<MotionOrder> getRequestedOrder();

         /**
          * Returns the current state
          */
         MotionState getMotionState() const;

         /**
          * Define a new timeout
          * @param timeout : in s
          */
         void setWorkTimeout(double timeout);

         /**
          * Define a new m_haltLinearVelocity
          * @param maxSpeed : in m/s
          */
         void setHaltLinearVelocity(double maxSpeed);

         /**
          * Define a new m_haltAngularVelocity
          * @param maxSpeed : in rad/s
          */
         void setHaltAngularVelocity(double maxSpeed);

    protected:
        /** Buffer for the next Order to do */
         shared_ptr<MotionOrder> m_requestedOrder;

        /** Current Order */
         shared_ptr<MotionOrder> m_currentOrder;

        /** Current state */
        MotionState m_state;

        /**  Buffer for order interruption sequence */
        bool m_interruptRequest;

        /** Date of the beginning of the order */
        double m_orderBeginTime;

        /** Timeout in WORK state to do an order */
        double m_workTimeout;

        /** threshold to consider the linear speed is low in m/s */
        double m_haltLinearVelocity;

        /** threshold to consider the angular speed is low  in rad/s*/
        double m_haltAngularVelocity;

        /**
         * do the required work in IDLE state.
         * the default order is NO_ORDER. When in no order
         * the robot should decrease speed gently
         */
        virtual void doIdle(arp_core::Pose currentPose);

        /**
         */
        virtual void doWork(arp_core::Pose currentPose);

        /**
         */
        virtual void doHalt(arp_core::Pose currentPose);
};

}

#endif /* ORDERSELECTOR_HPP_ */
