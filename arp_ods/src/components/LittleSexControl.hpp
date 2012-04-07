/*
 * LittleSexControl.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef LITTLESEXCONTROL_HPP_
#define LITTLESEXCONTROL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include "control/orders/orders.h"
#include <math/core>

namespace arp_ods
{

class LittleSexControl: public OdsTaskContext
{
    public:
        LittleSexControl(const std::string& name);
        void updateHook();

        /**
         * Use this to setup a new order
         * @return true on successfull change
         */
        bool ooSetOrder(shared_ptr<MotionOrder> order);

    protected:
        /**
         * Position courante du robot par rapport a la table dans le repère (??? XXX ??? )
         * C'est un eventPort qui schedule le composant.
         */
        RTT::InputPort< arp_math::EstimatedPose2D > inPosition;

        /**
         * Ordre a realiser. Il change lentement
         * il est possible de voir un changement avec readNewest() == NewData
         */
        RTT::InputPort< shared_ptr<MotionOrder> > inOrder;

        /**
         * This is the result of computation : the Twist that the robot should do
         */
        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;

        /**
         * This tells to the Order manager that we have finished the job.
         * We are waiting for a new order
         */
        RTT::OutputPort<bool> outOrderFinished;

        /**
         * This tells to the Order manager that we failed the job.
         * We are waiting an order cancellation
         */
        RTT::OutputPort<bool> outOrderInError;

        /**
         * Buffer for computed speed commands
         */
        arp_math::Twist2D attrComputedTwistCmd;

        /**
         * Buffer for current robot position
         */
        arp_math::EstimatedPose2D attrPosition;

        /**
         * Buffer for input order
         */
        shared_ptr<MotionOrder>  attrOrder;

        /**
         * Bufferize the inputs
         */
        void getInputs();

        /**
         * Check if wheel are blocked
         */
        void checkWheelBlocked();

        /**
         * Test if the current order is finished
         * This mean the order is in DONE or PASS (and configured as a pass order) momde.
         */
        bool isOrderFinished();

        /**
         * Publish the processed datas
         */
        void setOutputs();

        /**
         * Create ports/operation/Properties/Attributes/...
         */
        void createOrocosInterface();
};

} /* namespace arp_ods */
#endif /* LITTLESEXCONTROL_HPP_ */
