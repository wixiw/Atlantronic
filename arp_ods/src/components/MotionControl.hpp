/*
 * MotionControl.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include "control/orders/orders.h"
#include "control/orders/OrderFactory.hpp"
#include "control/ICRSpeedBuffer.hpp"
#include <math/core>

namespace arp_ods
{

class MotionControl: public OdsTaskContext
{
    public:
        MotionControl(const std::string& name);
        void updateHook();
        void stopHook();

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

        /** Measure of the current robot Twist */
        RTT::InputPort<arp_math::EstimatedICRSpeed> inCurrentICRSpeed;

        /** Geometric parameters */
        RTT::InputPort<arp_model::UbiquityParams> inParams;

        /** Period between 2 CanOpen Sync Messages*/
        RTT::InputPort<double> inCanPeriod;

        /**
         * This is the result of computation : the Twist that the robot should do
         */
        RTT::OutputPort<arp_math::ICRSpeed> outICRSpeedCmd;

        /**
         * Motioncontrol in approach mode
         */
        RTT::OutputPort<bool> outSmoothLocNeeded;

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

        /*
         * DEBUG
         */
        RTT::OutputPort<double> outDEBUG1;
        RTT::OutputPort<double> outDEBUG2;
        RTT::OutputPort<double> outDEBUG3;
        RTT::OutputPort<double> outDEBUG4;
        RTT::OutputPort<double> outDEBUG6;
        RTT::OutputPort<double> outDEBUG5;
        RTT::OutputPort<double>  outDEBUG7;
        RTT::OutputPort<double>  outDEBUG8;
        RTT::OutputPort<double>  outDEBUG9;
        RTT::OutputPort<double>  outDEBUG10;

        int m_norder;

        /**
         * Buffer for computed speed commands
         */
        arp_math::ICRSpeed attrComputedICRSpeedCmd;

        /**
         * Buffer for motion state
         */
        arp_math::UbiquityMotionState attrMotionState;

        /**
         * Period between 2 SYNC datas on the CanOpenBus
         * Take care : you also have another period information which ARDTaskContext::attrDt
         * which is the time between 2 calls of the updateHook.
         *
         * attrCanPeriod will be very stable but not related to your date of execution (you always execute after)
         * attrDt is the real delay between your last call and may be noisy if the OS introduce jitters.
         */
        double attrCanPeriod;

        /*
         * platform parameters
         */
        arp_model::UbiquityParams attrParams;

        /**
         * Buffer for input order
         */
        shared_ptr<orders::MotionOrder>  attrOrder;

        /**
         * Local variable to stock the max velocity that we were asked to follow
         */
        double attrVmax_asked;

        /**
         * Current order type
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
         * buffer of twist for replaying backward
         */
        ICRSpeedBuffer m_ICRSpeedBuffer;
        /*
         * function called to store the twist in the buffer
         */
        void storeICRSpeed();
        /*
         * minimum speed for twist that is stored
         */
        static const double MIN_STORED_TWIST_SPEED=0.005;

        /*
         * library for trajectory computations
         * must be instantiated at startup
         *
         */
        OnlineTrajectoryGenerator OTG;
};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
