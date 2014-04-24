/*
 * Dynamixel.hpp
 *
 *  Created on: 20 april 2014
 *      Author: wla
 */

#ifndef DYNAMIXEL_HPP_
#define DYNAMIXEL_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <arp_core/DynamixelState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

namespace arp_stm32
{

class Dynamixel: public Stm32TaskContext
{
    public:
        Dynamixel(const std::string& name);

        bool configureHook();
        void updateHook();

        /**
         * Set the goal position in rad of this dynamixel
         */
        RTT::InputPort<std_msgs::Float32> inPositionCmd;

        /**
         * Set the max torque of dynamixel in % of max value
         */
        RTT::InputPort<std_msgs::UInt8> inMaxTorqueAllowed;

        /**
         * Publish the internal state
         */
        RTT::OutputPort<arp_core::DynamixelState> outState;


    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        arp_core::DynamixelState attrState;
        double attrPositionCmd;
        int attrMaxTorque;

        double propPrecision;
        int propMaxTorque;
        int propId;
        int propDynamixelFamily;


        void sendPositionCmd(double position);
        void sendMaxTorqueCmd(int percentage);
        void sendPrecisionCmd(double precision);

        bool isTargetReached();
        bool isStucked();
        bool isConnected();
        double getPosition();
};

} /* namespace arp_stm32 */
#endif /* DYNAMIXEL_HPP_ */
