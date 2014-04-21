/*
 * PumpManager.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef PUMPMANAGER_HPP_
#define PUMPMANAGER_HPP_

#include "Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

class PumpManager: public Stm32TaskContext
{
    public:
        PumpManager(const std::string& name);

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
#endif /* PUMPMANAGER_HPP_ */
