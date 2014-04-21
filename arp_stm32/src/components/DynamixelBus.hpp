/*
 * DynamixelBus.hpp
 *
 *  Created on: 20 april 2014
 *      Author: wla
 */

#ifndef DYNAMIXELBUS_HPP_
#define DYNAMIXELBUS_HPP_

#include "Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

class DynamixelBus: public Stm32TaskContext
{
    public:
        DynamixelBus(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();

        /**
         * Scan dynamixels of all types
         */
        bool ooScanDynamixels();

        /**
         * Set new ID for dynamixel of type rx24F
         */
        bool ooDynamixelSetIdRX24F(int oldId, int newId);

        /**
         * Set new ID for dynamixel of type AX12
         */
        bool ooDynamixelSetIdAX12(int oldId, int newId);

        /**
         * Set the target of dynamixel with id to go to position
         */
        bool ooDynamixelSetPositionRX24F(int id, double position);

        /**
         * Set the target of dynamixel with id to go to position
         */
        bool ooDynamixelSetPositionAX12(int id, double position);

        /**
         * Set the max torque of dynamixel named id
         */
        bool ooDynamixelSetTorqueRX24F(int id, int percentage);

        /**
         * Set the max torque of dynamixel named id
         */
        bool ooDynamixelSetTorqueAX12(int id, int percentage);

        /**
         * Set the precision of dynamixel named id
         */
        bool ooDynamixelSetPrecisionRX24F(int id, double precision);

        /**
         * Set the precision of dynamixel named id
         */
        bool ooDynamixelSetPrecisionAX12(int id, double precision);

/****************************************************************
 * Interface ROS
 ****************************************************************/


    protected:
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;
        std::vector<bool> attrDynamixelTargetReachedRX24F;
        std::vector<bool> attrDynamixelTargetReachedAX12;
        std::vector<double> attrDynamixelPositionRX24F;
        std::vector<double> attrDynamixelPositionAX12;

        /**
         * Orocos Interface
         */

        RTT::OutputPort<bool> outRightCannonFingerTargetReached;
        RTT::OutputPort<bool> outRightCannonStockerTargetReached;
        RTT::OutputPort<double> outRightCannonFingerPosition;
        RTT::OutputPort<double> outRightCannonStockerPosition;
};

} /* namespace arp_stm32 */
#endif /* DYNAMIXELBUS_HPP_ */
