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

using namespace arp_core;
using namespace arp_math;

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

        InputPort<Twist2D> inTwistCmd;
        InputPort<UbiquityParams> inParams;

        InputPort<double> inLeftSteeringSpeedMeasure;
        InputPort<double> inRightSteeringSpeedMeasure;
        InputPort<double> inRearSteeringSpeedMeasure;

        OutputPort<double> outLeftDrivingSpeedCmd;
        OutputPort<double> outRightDrivingSpeedCmd;
        OutputPort<double> outRearDrivingSpeedCmd;
        OutputPort<double> outLeftSteeringPositionCmd;
        OutputPort<double> outRightSteeringPositionCmd;
        OutputPort<double> outRearSteeringPositionCmd;

        Twist2D attrTwistCmd;
};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
