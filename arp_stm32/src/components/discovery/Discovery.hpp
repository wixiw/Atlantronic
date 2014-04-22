/*
 * Discovery.hpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#ifndef DISCOVERY_HPP_
#define DISCOVERY_HPP_

#include <components/MotionScheduler.hpp>
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"
#include "linux/tools/qemu.h"
#include <arp_core/EmptyWithSuccess.h>

namespace arp_stm32
{

class Discovery: public arp_core::MotionScheduler
{
    public:
        Discovery(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();
        void cleanupHook();

        /**
         * Resets the Stm32 board
         */
        bool ooReset();

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /**
         * ROS wrapper on the HmlMonitor.ooReset operation
         */
        bool srvResetStm32(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);


    protected:
        static void robotItfCallbackWrapper(void* arg);
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

        std::string propDeviceName;
};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
