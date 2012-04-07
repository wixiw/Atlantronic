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
#include <arp_core/OmniCommand.h>
#include <arp_core/OmniOdo.h>
#include <arp_core/Start.h>
#include <arp_core/Pose.h>
#include <std_msgs/Bool.h>
//pour les services
#include <arp_core/SetPosition.h>
#include <arp_hml/SetMotorPower.h>
#include <arp_hml/ResetHml.h>
#include <arp_hml/GetVersion.h>

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
 * Oroso Interface
 ******************************************************/
        /** Buffured value of the input command for displaying purposes **/
        OmniCommand attrCurrentCmd;
        /** Buffered value of the odometers values for displaying purposes **/
        OmniOdo attrOdometers;

        /** Maximal delay beetween 2 received Differential commands. If this delay is overrun, a speed of 0 is sent on each motor. In s **/
        double propSpeedCmdMaxDelay;

        /**
         * Get enableDrive and disableDrive operation on motors
         */
        bool configureHook();

    	/**
    	 * Publish data from outside to HML, read HML data and present them in a formated way to outside.
    	 * In order :
    	 *  _ publish speed data
    	 *  _ read odometers value
    	 *  _ read color switch
    	 *  _ read start
    	 */
        void updateHook();

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetMotorPower;
        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetDrivingMotorPower;
        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetSteeringMotorPower;
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

        /** Speed command for left and right motor **/
        InputPort<arp_core::OmniCommand> inOmniCmd;

        /** Odometers value from left and right wheel assembled in an "Odo" Ros message **/
        OutputPort<arp_core::OmniOdo> outOdometryMeasures;

        /** Speed measures for left and right motor **/
        OutputPort<arp_core::OmniCommand> outOmniSpeedMeasure;

        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        OutputPort<arp_core::Start> outIoStart;

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

/*****************************************************************
 *  Interface with the INSIDE (hml !)
 *****************************************************************/

        /** HW value of the start switch. It is true when the start is in **/
        InputPort<bool> inIoStart;

        /** Real Position of the robot, given by the simulation */
        InputPort<Pose2D> inRealPosition;

        /** Value of the odometers in rad on the wheel axe **/
        InputPort<double> inLeftDrivingPosition;
        InputPort<double> inLeftDrivingPositionTime;
        InputPort<double> inRightDrivingPosition;
        InputPort<double> inRightDrivingPositionTime;
        InputPort<double> inRearDrivingPosition;
        InputPort<double> inRearDrivingPositionTime;
        InputPort<double> inLeftSteeringPosition;
        InputPort<double> inLeftSteeringPositionTime;
        InputPort<double> inRightSteeringPosition;
        InputPort<double> inRightSteeringPositionTime;
        InputPort<double> inRearSteeringPosition;
        InputPort<double> inRearSteeringPositionTime;

        /** Speed Value of the odometers in rad on the wheel axe **/
        InputPort<double> inLeftDrivingSpeedMeasure;
        InputPort<double> inRightDrivingSpeedMeasure;
        InputPort<double> inRearDrivingSpeedMeasure;
        InputPort<double> inLeftSteeringSpeedMeasure;
        InputPort<double> inRightSteeringSpeedMeasure;
        InputPort<double> inRearSteeringSpeedMeasure;

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
public:
        /** Use this to multiply a value given on the wheel's axe to get the value on the motor's axe **/
        static const double WHEEL_TO_MOTOR = 14.0;
        /** Use this to multiply a value given on the motor's axe to get the value on the wheel's axe **/
        static const double MOTOR_TO_WHEEL = 1.0/14.0;

protected:
        /** This holds the time of the last received differential command **/
        struct timespec m_lastCmdTimestamp;
        /** Is true when only one of the 2 speed has been received */
        bool m_receivedPartialPosition;

        /** Pointer on the HmlMonitor ooSetMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetMotorPower;
        /** Pointer on the HmlMonitor ooSetDrivingMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetDrivingMotorPower;
        /** Pointer on the HmlMonitor ooSetSteeringMotorPower Operation**/
        OperationCaller<bool(bool)> m_coSetSteeringMotorPower;
        /** Pointer on the HmlMonitor ooResetHml Operation**/
        OperationCaller<bool(void)> m_ooResetHml;
        /** Pointer on the HmlMonitor ooSetPosition Operation**/
        OperationCaller<bool(arp_math::Pose2D)> m_ooSetPosition;
        /** Pointer on the HmlMonitor coGetVersion Operation**/
        OperationCaller<std::string(void)> m_coGetVersion;

        /**
         * Get the differential command speed for both motor and dispatch it to them
         * If the 2 motors are not enabled, a null speed is forced
         */
        //void writeOmniCmd();

        /**
         * Read the odometers value and publish them together to outside
         */
        void readOdometers();

        /**
         * Read the start switch value and publish a go to the outside.
         */
        void readStart();

        /**
         * Read the drive enable value from HmlMonitor and publish them to Ros
         */
        void readDriveEnable();

        /**
         * Read the connectivity input and try to guess if the emergency stop is active
         */
        void readConnectivity();

        /**
         * Read the value of left and rigth motor and publish them together
         */
        void readSpeed();

        /**
         * Read if wheel are blocked
         */
        void readWheelBlocked();

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
