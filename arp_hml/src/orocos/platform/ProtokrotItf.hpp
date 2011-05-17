/*
 * ProtokrotItf.hpp
 *
 *  Created on: 10 april 2011
 *      Author: wla
 *
 *  This is the hardware interface published to the wonderfull open world. It *IS* hardware dependant and let the outside
 *  having a synthetic (and somehow synchronized) view of the current HML. This one is for Protokrot, the first ARD's robot,
 *  which is a nice differential robot.
 *
 *  Everything that comes in HML must go througth it
 *  Everything that goes outside HML must go througth it.
 *  It's HML Bigbrother.
 *
 *  Everything that is dependant on the Protokrot platform must come here !
 *  Everything that needs an information dependant on Protokrot platform must get it from here !
 */

#ifndef PROTOKROTITF_HPP_
#define PROTOKROTITF_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_hml/SetMotorPower.h>
#include <arp_hml/ResetHml.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sys/time.h>
#include "ros/ros.h"

using namespace arp_core;
using namespace std_msgs;

namespace arp_hml
{

    class ProtokrotItf: public HmlTaskContext
    {
    public:
    	ProtokrotItf(const std::string& name);

    protected:

/******************************************************
 * Oroso Interface
 ******************************************************/
        /** Buffured value of the input command for displaying purposes **/
        DifferentialCommand attrCurrentCmd;
        /** Buffered value of the odometers values for displaying purposes **/
        Odo attrOdometers;

        /** Gain on the left odometer to alter the position measure in rad on the wheel's axe.**/
        double propLeftOdometerGain;
        /** Gain on the right odometer to alter the position measure in rad on the wheel's axe.**/
        double propRightOdometerGain;
        /** Gain on the left motor's speed to alter the input command in rad/s on the wheel's axe.**/
        double propLeftSpeedGain;
        /** Gain on the right motor's speed to alter the input command in rad/s on the wheel's axe.**/
        double propRightSpeedGain;
        /** Maximal delay beetween 2 received Differential commands. If this delay is overrun, a speed of 0 is sent on each motor. In s **/
        double propSpeedCmdMaxDelay;

        /**
         * Returns a string containing Core version
         */
        string coGetCoreVersion();
        /**
         * Returns a string containing HML version
         */
        string coGetHmlVersion();

        /**
         * Defines if motor are powered or not.
         * @param poserOn : when set to true motor is powered and ready to move. If a null speed is provided, the motor is locked
         * as if brakes where enabled. When set to false there is no power in the motor which means that the motor is free to any move.
         * @param timeout : maximal time to wait for completion
         * @return : true if succeed on both motor. Failed if one or both have failed.
         */
        bool ooSetMotorPower(bool powerOn, double timeout);

        /**
         * Reset Can Nodes
         */
        bool ooResetHml();

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

        /** node handle to store the service advertiser srvResetHml**/
        ros::ServiceServer m_srvResetHml;

        /**
         * ROS wrapper on the ooSetPowerMotor operation
         */
        bool srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res);

        /**
         * ROS wrapper on the ooResetHml operation
         */
        bool srvResetHml(ResetHml::Request& req, ResetHml::Response& res);

/*****************************************************************
 *  Interface with OUTSIDE (master, ODS, RLU)
 *****************************************************************/

        /** Speed command for left and right motor **/
        InputPort<DifferentialCommand> inDifferentialCmd;

        /** Odometers value from left and right wheel assembled in an "Odo" Ros message **/
        OutputPort<Odo> outOdometryMeasures;

        /** Speed measures for left and right motor **/
        OutputPort<DifferentialCommand> outDifferentialMeasure;

        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        OutputPort<Start> outIoStart;

        /** Value of the color switch. It is "blue" when the switch button is on the front side of the robot)
          "red" when the button is on the rear side of the robot **/
        OutputPort<StartColor> outIoColorSwitch;

        /** Is true when HML thinks the emergency stop button is active **/
        OutputPort<Bool> outEmergencyStop;

        /** Is true when the 2 drives are enabled. Since this port is false, drive speed are forced to 0**/
        OutputPort<Bool> outDriveEnable;

/*****************************************************************
 *  Interface with the INSIDE (hml !)
 *****************************************************************/

        /** HW value of the start switch. It is true when the start is in **/
        InputPort<bool> inIoStart;

        /** HW value of the color switch. It is true when the color switch is on 1 **/
        InputPort<bool> inIoColorSwitch;

        /** Value of the left odometer in rad on the wheel axe **/
        InputPort<double> inLeftDrivingPosition;

        /** Value of the right odometer in rad on the wheel axe **/
        InputPort<double> inLeftSpeedMeasure;

        /** Value of the left speed in rad/s on the wheel axe **/
        InputPort<double> inRightSpeedMeasure;

        /** Value of the right speed in rad/s on the wheel axe **/
        InputPort<double> inRightDrivingPosition;

        /** Left drive soft enable state **/
        InputPort<bool> inLeftDriveEnable;

        /** Right drive soft enable state **/
        InputPort<bool> inRightDriveEnable;

        /** Left drive connectivity **/
        InputPort<bool> inLeftDriveConnected;

        /** Right drive connectivity **/
        InputPort<bool> inRightDriveConnected;

        /** Woodhead connectivity **/
        InputPort<bool> inWoodheadConnected;

        /** Speed command for the left motor in rad/s on the wheel axe **/
        OutputPort<double> outLeftSpeedCmd;

        /** Speed command for the right motor in rad/s on the wheel axe **/
        OutputPort<double> outRightSpeedCmd;


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

        /** Pointer on the LeftDrive ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableLeftDrive;
        /** Pointer on the Right ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRigthtDrive;
        /** Pointer on the LeftDrive ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableLeftDrive;
        /** Pointer on the Right ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRightDrive;
        /** Pointer in the Left ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetLeftOperationMode;
        /** Pointer in the Right ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRightOperationMode;
        /** Pointer in the Woodhead coReset Operation**/
        OperationCaller<bool(void)> m_coResetIo;
        /** Pointer in the LeftDriving coReset Operation **/
        OperationCaller<bool(void)> m_coResetLeftDriving;
        /** Pointer in the RightDriving coReset Operation **/
        OperationCaller<bool(void)> m_coResetRightDriving;

        /**
         * Get the differential command speed for both motor and dispatch it to them
         * If the 2 motors are not enabled, a null speed is forced
         */
        void writeDifferentialCmd();

        /**
         * Read the odometers value and publish them together to outside
         */
        void readOdometers();

        /**
         * Read the color switch value and publish it to outside in a string value
         */
        void readColorSwitch();

        /**
         * Read the start switch value and publish a go to the outside.
         */
        void readStart();

        /**
         * Read the drive enable value and merge the information for outside
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

    };

}

#endif
