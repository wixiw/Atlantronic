/*
 * Gpio.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include "Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

class Gpio: public Stm32TaskContext
{
    public:
        Gpio(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();

/****************************************************************
 * Interface ROS
 ****************************************************************/

    protected:
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;
};

} /* namespace arp_stm32 */
#endif /* GPIO_HPP_ */
