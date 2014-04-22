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
        RTT::InputPort<double> inPositionCmd;

        /**
         * Set the max torque of dynamixel in % of max value
         */
        RTT::InputPort<int> inMaxTorqueAllowed;

        /**
         * Set the precision of dynamixel in rad
         */
        RTT::InputPort<double> inPrecision;

        /**
         * Publish the dynamixel position in rad.
         */
        RTT::OutputPort<double> outPosition;

        /**
         * Inform if the dynamixel has reached the goal in the precision range
         */
        RTT::OutputPort<bool> outTargetReached;

        /**
         * Inform if the dynamixel is stucked and can't reach the goal
         */
        RTT::OutputPort<bool> outStuck;

    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        bool attrTargetReached;
        bool attrStucked;
        double attrPosition;

        double propPrecision;
        int propMaxTorque;
        int propId;
        int propDynamixelFamily;


        void sendPositionCmd(double position);
        void sendMaxTorqueCmd(int percentage);
        void sendPrecisionCmd(double precision);

        bool getTargetReached();
        bool getStucked();
        double getPosition();
};

} /* namespace arp_stm32 */
#endif /* DYNAMIXEL_HPP_ */
