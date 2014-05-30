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
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <math/core>

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
        bool startHook();
        void cleanupHook();

        /**
         * Resets the Stm32 board
         */
        virtual bool ooReset();

        /**
         * Power on the Stm32 board
         */
        bool ooPowerOn(bool on);

        RTT::OutputPort<arp_core::PowerStatusMsg> outPowerStatus;

        /**
         * Heartbeat update
         */
        RTT::InputPort<std_msgs::Empty> inHeartbeat;

        RTT::InputPort<std_msgs::Bool> inPowerRequest;

        // localization update
        RTT::InputPort<arp_math::EstimatedPose2D> inPose;

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
        bool isHeartBeatLost();
        double getBatteryVoltage();
        int getRawPowerData();
        int getRawGpioData();
        void sendHeartBeat();
        bool waitStm32BootDone(double timeout);

    protected:
        static void robotItfCallbackWrapper(void* arg);
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

        std::string propDeviceName;
        double propStm32BootDelay;

        int attrDebugGpio;
        int attrDebugPower;
        bool attrIsPowerOn;
        bool attrIsPowerDownDueToEndMatch;
        bool attrIsPowerDownDueToEmergencyStop;
        bool attrIsConnected;
        bool attrIsHeartbeatLost;
        double attrBatteryVoltage;

        std::vector<ros::ServiceServer> m_srvList;
};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
