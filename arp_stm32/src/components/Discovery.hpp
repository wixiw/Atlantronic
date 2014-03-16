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
        bool ooStopCalibration();

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

        /**
         * ROS wrapper on the HmlMonitor.ooStartCalibration operation
         */
        bool srvStartGyroCalibration(StartGyroCalibration::Request& req, StartGyroCalibration::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooStopCalibration operation
         */
        bool srvStopGyroCalibration(StopGyroCalibration::Request& req, StopGyroCalibration::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooSetPosition operation
         */
        bool srvSetGyroPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooReset operation
         */
        bool srvResetStm32(ResetStm32::Request& req, ResetStm32::Response& res);





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

};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
