/*
 * UbiquityItf.hpp
 *
 *  Created on: 03 Oct 2011
 *      Author: wla
 *
 *  This is the hardware interface published to the wonderfull open world. It *IS* hardware dependant and let the outside
 *  having a synthetic (and somehow synchronized) view of the current HML. This one is for UbiquityItf, the second ARD's robot,
 *  which is a nice omni-drive robot.
 *
 *  Everything that comes in HML must go througth it
 *  Everything that goes outside HML must go througth it.
 *  It's HML Bigbrother.
 *
 *  Everything that is dependant on the Ubiquity platform must come here !
 *  Everything that needs an information dependant on Ubiquity platform must get it from here !
 */

#ifndef UBIQUITYITF_HPP_
#define UBIQUITYITF_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <arp_core/OmniCommand.h>
#include <arp_core/OmniOdo.h>
#include <arp_core/Start.h>
#include <std_msgs/Bool.h>
#include <arp_hml/SetMotorPower.h>
#include <arp_hml/ResetHml.h>
#include <sys/time.h>
#include "ros/ros.h"

using namespace arp_core;
using namespace arp_hml;
using namespace std_msgs;

namespace arp_hml
{

    class UbiquityItf: public HmlTaskContext
    {
    public:
    	UbiquityItf(const std::string& name);

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
        InputPort<OmniCommand> inOmniCmd;

        /** Odometers value from left and right wheel assembled in an "Odo" Ros message **/
        OutputPort<OmniOdo> outOdometryMeasures;

        /** Speed measures for left and right motor **/
        OutputPort<OmniCommand> outOmniSpeedMeasure;

        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        OutputPort<Start> outIoStart;

        /** Is true when HML thinks the emergency stop button is active **/
        OutputPort<Bool> outEmergencyStop;

        /** Is true when the 2 drives are enabled. Since this port is false, drive speed are forced to 0**/
        OutputPort<Bool> outDriveEnable;

        /** Is true when wheel are blocked */
        OutputPort<Bool> outWheelBlocked;

/*****************************************************************
 *  Interface with the INSIDE (hml !)
 *****************************************************************/

        /** HW value of the start switch. It is true when the start is in **/
        InputPort<bool> inIoStart;

        /** Value of the traction odometers in rad on the wheel axe **/
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

        /** Value of the traction speeds in rad/s on the reductor output **/
        InputPort<double> inLeftDrivingSpeedMeasure;
        InputPort<double> inRightDrivingSpeedMeasure;
        InputPort<double> inRearDrivingSpeedMeasure;
        InputPort<double> inLeftSteeringSpeedMeasure;
        InputPort<double> inRightSteeringSpeedMeasure;
        InputPort<double> inRearSteeringSpeedMeasure;

        /** Drive soft enable state **/
        InputPort<bool> inLeftDrivingEnable;
        InputPort<bool> inRightDrivingEnable;
        InputPort<bool> inRearDrivingEnable;
        InputPort<bool> inLeftSteeringEnable;
        InputPort<bool> inRightSteeringEnable;
        InputPort<bool> inRearSteeringEnable;

        /** drive connectivity **/
        InputPort<bool> inLeftDrivingConnected;
        InputPort<bool> inRightDrivingConnected;
        InputPort<bool> inRearDrivingConnected;
        InputPort<bool> inLeftSteeringConnected;
        InputPort<bool> inRightSteeringConnected;
        InputPort<bool> inRearSteeringConnected;

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

        /** Woodhead connectivity **/
        InputPort<bool> inWoodheadInConnected;
        InputPort<bool> inWoodheadOutConnected;

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

        /** Pointer on the LeftDrive ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableLeftDriving;
        /** Pointer on the RightDriveooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRigthtDriving;
        /** Pointer on the RearDrive ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRearDriving;
        /** Pointer on the Left Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableLeftSteering;
        /** Pointer on the Right Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRigthtSteering;
        /** Pointer on the Rear Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRearSteering;

        /** Pointer on the LeftDrive ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableLeftDriving;
        /** Pointer on the Right ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRightDriving;
        /** Pointer on the RearDrive ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRearDriving;
        /** Pointer on the Left Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableLeftSteering;
        /** Pointer on the Right Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRightSteering;
        /** Pointer on the Rear Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRearSteering;

        /** Pointer in the Left driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetLeftDrivingOperationMode;
        /** Pointer in the Right driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRightDrivingOperationMode;
        /** Pointer in the Right driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRearDrivingOperationMode;
        /** Pointer in the Left steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetLeftSteeringOperationMode;
        /** Pointer in the Right steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRightSteeringOperationMode;
        /** Pointer in the Right steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRearSteeringOperationMode;

        /** Pointer in the Woodhead 8 out coReset Operation**/
        OperationCaller<bool(void)> m_coResetWoodheadOut;
        /** Pointer in the Woodhead 8 in coReset Operation**/
        OperationCaller<bool(void)> m_coResetWoodheadIn;
        /** Pointer in the LeftDriving coReset Operation **/
        OperationCaller<bool(void)> m_coResetLeftDriving;
        /** Pointer in the RightDriving coReset Operation **/
        OperationCaller<bool(void)> m_coResetRightDriving;
        /** Pointer in the RearDriving coReset Operation **/
        OperationCaller<bool(void)> m_coResetRearDriving;
        /** Pointer in the LeftSteering coReset Operation **/
        OperationCaller<bool(void)> m_coResetLeftSteering;
        /** Pointer in the RightSteering coReset Operation **/
        OperationCaller<bool(void)> m_coResetRightSteering;
        /** Pointer in the RearSteering coReset Operation **/
        OperationCaller<bool(void)> m_coResetRearSteering;

        /**
         * Get the differential command speed for both motor and dispatch it to them
         * If the 2 motors are not enabled, a null speed is forced
         */
        void writeOmniCmd();

        /**
         * Read the odometers value and publish them together to outside
         */
        void readOdometers();

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

        /**
         * Read if wheel are blocked
         */
        void readWheelBlocked();
    };

}

#endif
