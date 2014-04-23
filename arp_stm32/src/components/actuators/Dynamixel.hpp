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

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

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
         * Set the precision of dynamixel in rad
         */
        RTT::InputPort<std_msgs::Float32> inPrecision;

        /**
         * Publish the dynamixel position in rad.
         */
        RTT::OutputPort<std_msgs::Float32> outPosition;

        /**
         * Inform if the dynamixel has reached the goal in the precision range
         */
        RTT::OutputPort<std_msgs::Bool> outTargetReached;

        /**
         * Inform if the dynamixel is stucked and can't reach the goal
         */
        RTT::OutputPort<std_msgs::Bool> outStucked;

        /**
         * Inform if the dynamixel is connected to the bus
         */
        RTT::OutputPort<std_msgs::Bool> outConnected;

    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        std_msgs::Bool attrTargetReached;
        std_msgs::Bool attrStucked;
        std_msgs::Bool attrConnected;
        std_msgs::Float32 attrPosition;

        double propPrecision;
        int propMaxTorque;
        int propId;
        int propDynamixelFamily;


        void sendPositionCmd(std_msgs::Float32 position);
        void sendMaxTorqueCmd(std_msgs::UInt8 percentage);
        void sendPrecisionCmd(std_msgs::Float32 precision);

        bool isTargetReached();
        bool isStucked();
        bool isConnected();
        double getPosition();
};

} /* namespace arp_stm32 */
#endif /* DYNAMIXEL_HPP_ */
