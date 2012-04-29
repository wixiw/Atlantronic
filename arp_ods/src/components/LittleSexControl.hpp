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
         * Use this operation to setup a new order
         * @return true on successfull change
         */
        bool ooSetOrder(shared_ptr<orders::MotionOrder> order);

        /**
         * Use this opration to setup a new max speed
         * @param vmax : new max speed in m/s
         * @return true on successfull change aka the max speed sent is positive
         */
        bool ooSetVMax(double vmax);

    protected:
        /**
         * Position courante du robot par rapport a la table dans le repère (??? XXX ??? )
         * C'est un eventPort qui schedule le composant.
         */
        RTT::InputPort< arp_math::EstimatedPose2D > inPosition;

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
        shared_ptr<orders::MotionOrder>  attrOrder;

        /**
         * Local variable to stock the max velocity that we were aked to follow
         */
        double attrVMax;

        /**
         * Current order type to debug
         */
        std::string attrCurrentOrder;

        /**
         * Bufferize the inputs
         */
        void getInputs();


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


        /*
         * time of last call
         */
        double m_oldTime;
        /*
         * get the dt since last call
         */
        double getDt();
};

} /* namespace arp_ods */
#endif /* LITTLESEXCONTROL_HPP_ */
