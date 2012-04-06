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
        /** Clock port which trigger our activity. contains the Twist command we ha to try to do on the robot*/
        InputPort<arp_math::Twist2D> inTwistCmd;
        InputPort<arp_model::UbiquityParams> inParams;

		InputPort<arp_math::Twist2D> inCurrentTwist;
        InputPort<arp_model::MotorState> inMotorState;

        OutputPort<double> outLeftDrivingVelocityCmd;
        OutputPort<double> outRightDrivingVelocityCmd;
        OutputPort<double> outRearDrivingVelocityCmd;

        OutputPort<double> outLeftSteeringPositionCmd;
        OutputPort<double> outRightSteeringPositionCmd;
        OutputPort<double> outRearSteeringPositionCmd;

        arp_math::Twist2D attrTwistCmd;
        arp_math::Twist2D attrCurrentTwist;
        arp_math::Twist2D attrAcceptableTwist;
        arp_model::MotorState attrMotorsCurrentState;


};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
