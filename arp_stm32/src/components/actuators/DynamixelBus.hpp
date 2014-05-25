/*
 * DynamixelBus.hpp
 *
 *  Created on: 20 april 2014
 *      Author: wla
 */

#ifndef DYNAMIXELBUS_HPP_
#define DYNAMIXELBUS_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
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
        bool ooScanRx24F();
        bool ooScanAx12();

        /**
         * Set Id="newId" for dynamixel of type "family" named "oldId"
         */
        bool ooSetId(int family, int oldId, int newId);

        /**
         * Set a new baudrate for the bus of the family type
         */
        bool ooSetBusBaudRate(int family, int baudrate);

        /**
         * Set a baudrate to 1M for the dynamixel id of the family type type
         */
        bool ooSetDynamixelBaudRate(int family, int id);

/****************************************************************
 * Interface ROS
 ****************************************************************/


    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;
};

} /* namespace arp_stm32 */
#endif /* DYNAMIXELBUS_HPP_ */
