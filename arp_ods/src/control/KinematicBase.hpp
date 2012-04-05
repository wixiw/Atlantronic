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
#include <models/UbiquityParams.hpp>
#include <models/UbiquityKinematics.hpp>

namespace arp_ods
{

class KinematicBase: public OdsTaskContext
{
    public:
        KinematicBase(const std::string& name);

        void updateHook();

    protected:
        /** Clock port which trigger our activity. It contains the time at which the input data are supposed to be calculated*/
        InputPort<timespec> inClock;

        InputPort<arp_math::Twist2D> inTwistCmd;
        InputPort<arp_core::UbiquityParams> inParams;

		InputPort<arp_math::Twist2D> inCurrentTwist;

        InputPort<double> inLeftSteeringVelocityMeasure;
        InputPort<double> inRightSteeringVelocityMeasure;
        InputPort<double> inRearSteeringVelocityMeasure;
        
        InputPort<double> inLeftSteeringPositionMeasure;
        InputPort<double> inRightSteeringPositionMeasure;
        InputPort<double> inRearSteeringPositionMeasure;
        
        InputPort<double> inLeftDrivingVelocityMeasure;
        InputPort<double> inRightDrivingVelocityMeasure;
        InputPort<double> inRearDrivingVelocityMeasure;

        OutputPort<double> outLeftDrivingVelocityCmd;
        OutputPort<double> outRightDrivingVelocityCmd;
        OutputPort<double> outRearDrivingVelocityCmd;

        OutputPort<double> outLeftSteeringPositionCmd;
        OutputPort<double> outRightSteeringPositionCmd;
        OutputPort<double> outRearSteeringPositionCmd;

        arp_math::Twist2D attrTwistCmd;
        arp_math::Twist2D attrCurrentTwist;
        arp_math::Twist2D attrAcceptableTwist;
        arp_core::MotorState attrMotorsCurrentState;


};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
