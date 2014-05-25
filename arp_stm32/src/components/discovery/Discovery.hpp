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
#include <arp_stm32/SetPowerSrv.h>
#include <arp_core/PowerStatusMsg.h>


namespace arp_stm32
{

class Discovery: public arp_core::MotionScheduler
{
    public:
        Discovery(const std::string& name);
        ~Discovery();

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

        /**
         * Power on the Stm32 board
         */
        bool ooPowerOn(bool on);

        RTT::OutputPort<arp_core::PowerStatusMsg> outPowerStatus;

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /**
         * ROS wrapper on the HmlMonitor.ooReset operation
         */
        bool srvResetStm32(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooPowerOn operation
         */
        bool srvPowerOn(SetPowerSrv::Request& req, SetPowerSrv::Response& res);

/****************************************************************
 * Interface Stm32
 ****************************************************************/

        bool isPowerOn();
        bool isEmergencyStopActive();
        bool isUnderVoltageErrorActive();
        bool isPowerAllowedByStragety();
        bool isPowerShutdownAtEndOfMatch();
        double getBatteryVoltage();
        int getRawPowerData();
        int getRawGpioData();

    protected:
        static void robotItfCallbackWrapper(void* arg);
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

        std::string propDeviceName;

        int attrDebugGpio;
        int attrDebugPower;
        bool attrIsPowerOn;
        double attrBatteryVoltage;
};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
