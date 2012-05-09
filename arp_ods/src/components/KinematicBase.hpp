/*
 * MotionControl.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>
#include <models/core>

namespace arp_ods
{

class KinematicBase: public OdsTaskContext
{
    public:
        KinematicBase(const std::string& name);

        void updateHook();

    protected:
        /** define if robot has been blocked for timeout time*/
        bool attrRobotBlockedTimeout;
        /** time I began to block */
        double attrBlockTime;
        /** Pour le debug uniquement, permet de commander des 0 purs aux moteurs de directions pour "détourer" */
        bool attrGoToZero;
        arp_math::Twist2D attrTwistCmd;
        arp_math::Twist2D attrCurrentTwist;
        arp_math::Twist2D attrAcceptableTwist;
        arp_model::MotorState attrMotorStateCommand;
        /**Internal model feedback, for debug info only */
        arp_model::TurretState attrTurretState;
        arp_model::MotorState attrMotorsCurrentState;
        arp_model::UbiquityParams attrParams;
        double attrQuality;

        double propMinSpeed;
        /** timeout for the robot to be considered blocked */
        double propRobotBlockedTimeout;
        /** Distance maximale du twist mesuré au twist commandé en m/s avant de détecter un blocage*/
        double propMaxSpeedDiff;




        /** Clock port which trigger our activity. contains the Twist command we ha to try to do on the robot*/
        RTT::InputPort<arp_math::Twist2D> inTwistCmd;
        /** Measure of the current robot Twist */
        RTT::InputPort<arp_math::EstimatedTwist2D> inCurrentTwist;
        /** Measures of the motor state. In a way it is redundant with inCurrentTwist but as the state does'nt
         match the same size, I prefer getting them from HML instead of computed it from inTwist. And to finish wit
         we need internally Turret speeds anyway to compute couplings so it doesn't cost much to take the entire state */
        RTT::InputPort<arp_model::MotorState> inMotorState;
        /** Geometric parameters */
        RTT::InputPort<arp_model::UbiquityParams> inParams;

        RTT::OutputPort<double> outLeftDrivingVelocityCmd;
        RTT::OutputPort<double> outRightDrivingVelocityCmd;
        RTT::OutputPort<double> outRearDrivingVelocityCmd;

        RTT::OutputPort<double> outLeftSteeringPositionCmd;
        RTT::OutputPort<double> outRightSteeringPositionCmd;
        RTT::OutputPort<double> outRearSteeringPositionCmd;

        RTT::OutputPort<double> outFiltrationFeedback;
        RTT::OutputPort<arp_math::Twist2D> outFilteredTwist;
        RTT::OutputPort<bool> outRobotBlocked;

        /**
         * Utility functions to help the developper te separate :
         * _ input buffering (local snapshot to prevent data desynchronisation)
         * _ cycle compuation
         * _ result publishing (gathered to be as closely shyncronized)
         *
         * It also forces the developper to use member variables to store interface datas that can be
         * exposed to the orocos Interface
         */
        void getInputs();
        void run();
        void setOutputs();

        /**
         * Utility function to hide at the end of the file the verbose Orocos interface declarations
         */
        void createOrocosInterface();

        /**
         * Check if wheel are blocked
         */
        void checkRobotBlocked();
        /*
         * check if measured twist and commanded twist are ok
         */
        bool consistencyMeasuredvsCommanded();

};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
