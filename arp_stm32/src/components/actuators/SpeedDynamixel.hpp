/*
 * Dynamixel.hpp
 *
 *  Created on: 20 april 2014
 *      Author: wla
 */

#ifndef SPEED_DYNAMIXEL_HPP_
#define SPEED_DYNAMIXEL_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <arp_core/DynamixelState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

namespace arp_stm32
{

class SpeedDynamixel: public Stm32TaskContext
{
    public:
        SpeedDynamixel(const std::string& name);

        bool configureHook();
        void updateHook();

        /**
         * Set the goal position in rad of this dynamixel
         */
        RTT::InputPort<std_msgs::Float32> inSpeedCmd;

        /**
         * Publish the internal state
         */
        RTT::OutputPort<arp_core::DynamixelState> outState;


    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        arp_core::DynamixelState attrState;
        double attrSpeedCmd;
        bool attrConnected;
        double attrPositionMeasure;

        int propId;
        int propDynamixelFamily;

        void sendSpeedCmd(double speed);

        bool isConnected();
        double getPosition();
};

} /* namespace arp_stm32 */
#endif /* SPEED_DYNAMIXEL_HPP_ */
