/*
 * RosHmlItf.hpp
 *
 *  Created on: 03 Oct 2011
 *      Author: wla
 *
 *  This is the hardware interface published to the wonderfull open world. It *IS* hardware dependant and let the outside
 *  having a synthetic (and somehow synchronized) view of the current HML. This one is for UbiquityItf, the second ARD's robot,
 *  which is a nice omni-drive robot.
 *
 *  Everything that comes in HML from outside of Orocos must go througth it
 *  Everything that goes outside HML from outside Oof rocos must go througth it.
 *  It's HML Bigbrother.
 *
 *  It is allowed to connect directly from Orocos to Hml component since composites components doesn't exsits yet.
 *  The HmlMonitor is another part of the Interface, espacially for the Orocos world.
 *
 */

#ifndef ROSHMLITF_HPP_
#define ROSHMLITF_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <sys/time.h>
#include "ros/ros.h"
#include <math/core>
//pour les ports
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_core/Obstacle.h>
#include <arp_core/Pose.h>
#include <std_msgs/Bool.h>
//pour les services
#include <arp_core/SetPosition.h>
#include <arp_hml/SetMotorPower.h>
#include <arp_hml/SetMotorMode.h>
#include <arp_hml/ResetHml.h>
#include <arp_hml/GetVersion.h>

#include <models/UbiquityStates.hpp>

using namespace arp_core;
using namespace arp_math;
using namespace arp_hml;
using namespace std_msgs;

namespace arp_hml
{

    class RosHmlItf: public HmlTaskContext
    {
    public:
    	RosHmlItf(const std::string& name);

    protected:

/******************************************************
 * Orocos Interface
 ******************************************************/


        bool configureHook();

    	/**
    	 * Publish data from outside to HML, read HML data and present them in a formated way to outside.
    	 * In order :
    	 *  _ read color switch
    	 *  _ read start
    	 */
        void updateHook();

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetMotorPower;
        /** node handle to store the service advertiser srvSetDrivingMotorPower**/
        ros::ServiceServer m_srvSetDrivingMotorPower;
        /** node handle to store the service advertiser srvSetSteeringMotorPower**/
        ros::ServiceServer m_srvSetSteeringMotorPower;
        /** node handle to store the service advertiser srvSetDrivingOperationMode**/
        ros::ServiceServer m_srvSetDrivingOperationMode;
        /** node handle to store the service advertiser srvSetSteeringOperationMode**/
        ros::ServiceServer m_srvSetSteeringOperationMode;
        /** node handle to store the service advertiser srvResetHml**/
        ros::ServiceServer m_srvResetHml;
        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetRealSimulPosition;
        /** node handle to store the service advertiser srvResetHml**/
        ros::ServiceServer m_srvGetVersion;

        /**
         * ROS wrapper on the HmlMonitor.ooSetPowerMotor operation
         */
        bool srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res);

        /**
         * ROS wrapper on the ooSetPowerMotor operation
         */
        bool srvSetDrivingMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res);

        /**
         * ROS wrapper on the ooSetPowerMotor operation
         */
        bool srvSetSteeringMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res);

        /**
         * ROS wrapper on the ooSetPowerMotor operation
         */
        bool srvSetDrivingOperationMode(SetMotorMode::Request& req, SetMotorMode::Response& res);

        /**
         * ROS wrapper on the ooSetPowerMotor operation
         */
        bool srvSetSteeringOperationMode(SetMotorMode::Request& req, SetMotorMode::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooResetHml operation
         */
        bool srvResetHml(ResetHml::Request& req, ResetHml::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooSetRealSimulPosition operation
         */
        bool srvSetRealSimulPosition(SetPosition::Request& req, SetPosition::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.coGethmlVersion operation
         */
        bool srvGetVersion(GetVersion::Request& req, GetVersion::Response& res);

/*****************************************************************
 *  Interface with OUTSIDE (master, ODS, RLU)
 *****************************************************************/

        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        OutputPort<arp_core::Start> outIoStart;
        /** Value of the color switch. true when ?? **/
        OutputPort<arp_core::StartColor> outIoStartColor;

        OutputPort<arp_core::Obstacle> outFrontLeftObstacle;
        OutputPort<arp_core::Obstacle> outFrontRightObstacle;
        OutputPort<arp_core::Obstacle> outRearObstacle;

        /** Is true when HML thinks the emergency stop button is active **/
        OutputPort<std_msgs::Bool> outEmergencyStop;

        /** Output of Enable value of motors to publish to Ros **/
        OutputPort<std_msgs::Bool> outDrivingMotorsEnable;
        OutputPort<std_msgs::Bool> outSteeringMotorsEnable;
        OutputPort<std_msgs::Bool> outMotorsEnable;

        /** Is true when wheel are blocked */
        OutputPort<std_msgs::Bool> outWheelBlocked;

        /** Real Position of the robot, given by the simulation */
        OutputPort<Pose> outRealPosition;

        /** Homing done */
        OutputPort<std_msgs::Bool> outIsHomingDone;

        /** Order from simulation to block the wheels         */
        InputPort<std_msgs::Bool> inBlockRobot;

/*****************************************************************
 *  Interface with the INSIDE (hml !)
 *****************************************************************/

        /** HW value of the start switch. It is true when the start is in **/
        InputPort<bool> inIoStart;
        /** HW value of the color switch. It is true when color is red **/
        InputPort<bool> inIoStartColor;
        /** Obstacles */
        InputPort<bool> inIoFrontLeftObstacle;
        InputPort<bool> inIoFrontRightObstacle;
        InputPort<bool> inIoRearObstacle;

        /** Real Position of the robot, given by the simulation */
        InputPort<Pose2D> inRealPosition;

        /** State of the 6 mobile base motors **/
        InputPort<arp_model::MotorState> inMotorMeasures;

        /** µIs true when the 3 steering motor has done their homing */
        InputPort<bool> inIsHomingDone;

        /** Input of Enable value of motors to publish to Ros **/
        InputPort<bool> inDrivingMotorsEnable;
        InputPort<bool> inSteeringMotorsEnable;
        InputPort<bool> inMotorsEnable;

        /** Blocage roue */
        InputPort<bool> inLeftDrivingBlocked;
        InputPort<bool> inRightDrivingBlocked;
        InputPort<bool> inRearDrivingBlocked;
        InputPort<bool> inLeftSteeringBlocked;
        InputPort<bool> inRightSteeringBlocked;
        InputPort<bool> inRearSteeringBlocked;

        /** order of blocking robot */
        OutputPort<bool> outBlockRobot;

        /** Speed command for the motors in rad/s on the reductor output axe **/
        OutputPort<double> outLeftDrivingSpeedCmd;
        OutputPort<double> outRightDrivingSpeedCmd;
        OutputPort<double> outRearDrivingSpeedCmd;

        /** Position command for the motors in rad on the reductor output axe **/
        OutputPort<double> outLeftSteeringPositionCmd;
        OutputPort<double> outRightSteeringPositionCmd;
        OutputPort<double> outRearSteeringPositionCmd;

        /** Torque command for the motors in Nm on the reductor output axe **/
        OutputPort<double> outLeftDrivingTorqueCmd;
        OutputPort<double> outRightDrivingTorqueCmd;
        OutputPort<double> outRearDrivingTorqueCmd;
        OutputPort<double> outLeftSteeringTorqueCmd;
        OutputPort<double> outRightSteeringTorqueCmd;
        OutputPort<double> outRearSteeringTorqueCmd;


/**************************************************************
 * Internals
 **************************************************************/

protected:
        /** Pointer on the HmlMonitor ooSetMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetMotorPower;
        /** Pointer on the HmlMonitor ooSetDrivingMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetDrivingMotorPower;
        /** Pointer on the HmlMonitor ooSetSteeringMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetSteeringMotorPower;
        /** Pointer on the HmlMonitor ooSetDrivingOperationMode Operation**/
        OperationCaller<bool(std::string)>  m_ooSetDrivingOperationMode;
        /** Pointer on the HmlMonitor ooSetSteeringOperationMode Operation**/
        OperationCaller<bool(std::string)>  m_ooSetSteeringOperationMode;
        /** Pointer on the HmlMonitor ooResetHml Operation**/
        OperationCaller<bool(void)> m_coResetHml;
        /** Pointer on the HmlMonitor ooSetPosition Operation**/
        OperationCaller<bool(arp_math::Pose2D)> m_ooSetPosition;
        /** Pointer on the HmlMonitor coGetVersion Operation**/
        OperationCaller<std::string(void)> m_coGetVersion;

        /**
         * Read Io fro woodhead and publish a go to the outside.
         */
        void readIo();

        /**
         * Read the drive enable value from HmlMonitor and publish them to Ros
         */
        void readDriveEnable();

        /**
         * Read the connectivity input and try to guess if the emergency stop is active
         */
        void readConnectivity();

        /**
         * Read if wheel are blocked
         */
        void readWheelBlocked();
        /*
         * read if someone asked for a robot blockage
         */
        void readBlockRobot();

        /**
         * create the Orocos interface (creation of ports, commands, attributes, ...)
         */
        void createOrocosInterface();

        /**
         * create the ROS interface (creation of services, topics,...)
         */
        void createRosInterface();
    };

}

#endif
