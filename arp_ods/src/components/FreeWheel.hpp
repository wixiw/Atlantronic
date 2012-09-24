/*
 * FreeWheel.hpp
 *
 *  Created on: 29 Sept, 2012
 *      Author: ard
 */

#ifndef FREEWHEEL_HPP_
#define FREEWHEEL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>
#include <models/core>

#include <std_msgs/Bool.h>

namespace arp_ods
{

class FreeWheel: public OdsTaskContext
{
    public:
        FreeWheel(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        arp_math::Twist2D attrTwistCmd;

        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;

        RTT::InputPort<double> inLeftWheelSpeed;
        RTT::InputPort<double> inRightWheelSpeed;

        //mesure actuelle
        RTT::InputPort<arp_model::UbiquityParams>inParams;
        RTT::InputPort<bool> inPower;
        RTT::InputPort<std_msgs::Bool> inBootUpDone;
        RTT::OperationCaller<bool(bool)> m_coSetMotorPower;

        void createOrocosItf();
};

} /* namespace arp_ods */
#endif /* FREEWHEEL_HPP_ */
