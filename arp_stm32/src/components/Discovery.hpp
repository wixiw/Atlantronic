/*
 * Discovery.hpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#ifndef DISCOVERY_HPP_
#define DISCOVERY_HPP_

#include "Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <arp_stm32/StartGyroCalibration.h>
#include <arp_stm32/StopGyroCalibration.h>
#include <arp_stm32/SetGyroPosition.h>
#include <arp_stm32/ResetStm32.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>

#include <arp_core/EmptyWithSuccess.h>

namespace arp_stm32
{

class DiscoveryLock
{
    public:
        DiscoveryLock(pthread_mutex_t* mutex);
        ~DiscoveryLock();

        enum eLockResult
        {
            SUCCEED = 0, FAILED = 1
        };

        //lock is true if the lock failed
        eLockResult lock();
        void unlock();

    private:
        pthread_mutex_t* m_mutex;
};

class Discovery: public Stm32TaskContext
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
         * Ask the gyrometer to freeze its position and to start the calibration process
         * returns true on success
         */
        bool ooStartCalibration();

        /**
         * Ask the gyrometer to stop the calibration process and to re-publish position datas
         * returns true on success
         */
        bool ooStopCalibration(double newAngle);

        /**
         * Force a new gyrometer position, in rad
         */
        bool ooSetPosition(double newAngle);

        /**
         * Force new calib
         */
        bool ooSetCalibrationValues(double scale, double bias, double dead_zone);

        /**
         * Resets the Stm32 board
         */
        bool ooReset();

        /**
         * Informs the stm32 that the next start withdraw sill be the match begining
         */
        bool ooEnableStart();

        /**
         * Scan dynamixels of type rx24F
         */
        bool ooScanDynamixelRX24F();

        /**
         * Scan dynamixels of type rx24F
         */
        bool ooScanDynamixelAX12();

        /**
         * Set new ID for dynamixel of type rx24F
         */
        bool ooDynamixelSetIdRX24F(int oldId, int newId);

        /**
         * Set new ID for dynamixel of type AX12
         */
        bool ooDynamixelSetIdAX12(int oldId, int newId);

        /**
         * Set the target of dynamixel with id to go to position
         */
        bool ooDynamixelSetPositionRX24F(int id, double position);

        /**
         * Set the target of dynamixel with id to go to position
         */
        bool ooDynamixelSetPositionAX12(int id, double position);

        /**
         * Set the max torque of dynamixel named id
         */
        bool ooDynamixelSetTorqueRX24F(int id, int percentage);

        /**
         * Set the max torque of dynamixel named id
         */
        bool ooDynamixelSetTorqueAX12(int id, int percentage);

        /**
         * Set the precision of dynamixel named id
         */
        bool ooDynamixelSetPrecisionRX24F(int id, double precision);

        /**
         * Set the precision of dynamixel named id
         */
        bool ooDynamixelSetPrecisionAX12(int id, double precision);

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /** node handle to store the service advertiser srvStartGyroCalibration**/
        ros::ServiceServer m_srvStartGyroCalibration;
        /** node handle to store the service advertiser srvStopGyroCalibration**/
        ros::ServiceServer m_srvStopGyroCalibration;
        /** node handle to store the service advertiser srvSetGyroPosition**/
        ros::ServiceServer m_srvSetGyroPosition;
        /** node handle to store the service advertiser m_srvResetStm32**/
        ros::ServiceServer m_srvResetStm32;
        /** node handle to store the service advertiser m_srvResetStm32**/
        ros::ServiceServer m_srvEnableStart;

        /**
         * ROS wrapper on the HmlMonitor.ooStartCalibration operation
         */
        bool srvStartGyroCalibration(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooStopCalibration operation
         */
        bool srvStopGyroCalibration(SetGyroPosition::Request& req, SetGyroPosition::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooSetPosition operation
         */
        bool srvSetGyroPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooReset operation
         */
        bool srvResetStm32(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.oo operation
         */
        bool srvEnableStart(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);



    protected:
        static void robotItfCallbackWrapper(void* arg);
        void robotItfUpdated();
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface m_robotItf;
        int attrGyrometerRawData;
        double attrGyrometerVelocity;
        double attrGyrometerVelocityDegree;
        double attrGyrometerAngleEuler;
        double attrGyrometerAngleEulerDegree;
        double attrGyrometerAngleSimpson;
        double attrGyrometerAngleSimpsonDegree;
        bool attrStartPlugged;
        bool attrStartColor;
        std::vector<bool> attrDynamixelTargetReachedRX24F;
        std::vector<bool> attrDynamixelTargetReachedAX12;
        std::vector<double> attrDynamixelPositionRX24F;
        std::vector<double> attrDynamixelPositionAX12;

        /**
         * Orocos Interface
         */

        //Gyrometer position in radian in [-pi;pi[
        RTT::OutputPort<double> outGyrometerVelocity;
        RTT::OutputPort<double> outGyrometerVelocityDegree;
        RTT::OutputPort<double> outGyrometerAngleEuler;
        RTT::OutputPort<double> outGyrometerAngleEulerDegree;
        RTT::OutputPort<double> outGyrometerAngleSimpson;
        RTT::OutputPort<double> outGyrometerAngleSimpsonDegree;
        RTT::OutputPort<double> outGyrometerRawData;
        RTT::OutputPort<bool> outRightCannonFingerTargetReached;
        RTT::OutputPort<bool> outRightCannonStockerTargetReached;
        RTT::OutputPort<double> outRightCannonFingerPosition;
        RTT::OutputPort<double> outRightCannonStockerPosition;

        //Start/Color
        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        RTT::OutputPort<arp_core::Start> outIoStart;
        /** Value of the color switch. true when ?? **/
        RTT::OutputPort<arp_core::StartColor> outIoStartColor;

};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
