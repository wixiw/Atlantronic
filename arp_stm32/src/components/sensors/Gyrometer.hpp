/*
 * Gyrometer.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef GYROMETER_HPP_
#define GYROMETER_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"
#include <list>

#include <arp_msgs/SetGyroPositionSrv.h>
#include <arp_msgs/EmptyWithSuccessSrv.h>

namespace arp_stm32
{

class Gyrometer: public Stm32TaskContext
{
    public:
        Gyrometer(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();

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

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /**
         * ROS wrapper on the HmlMonitor.ooStartCalibration operation
         */
        bool srvStartCalibration(arp_msgs::EmptyWithSuccessSrv::Request& req, arp_msgs::EmptyWithSuccessSrv::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooStopCalibration operation
         */
        bool srvStopCalibration(arp_msgs::SetGyroPositionSrv::Request& req, arp_msgs::SetGyroPositionSrv::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooSetPosition operation
         */
        bool srvSetPosition(arp_msgs::SetGyroPositionSrv::Request& req, arp_msgs::SetGyroPositionSrv::Response& res);


    protected:
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

        /** This is a dummy var to hold service pointers so that they doesn't ket automatically cleared*/
        std::list<ros::ServiceServer> m_serviceList;

        int         attrRawData;
        double      attrVelocity;
        double      attrVelocityDegree;
        double      attrAngleEuler;
        double      attrAngleEulerDegree;
        double      attrAngleSimpson;
        double      attrAngleSimpsonDegree;

        //TODO comments !!
        double      propCalibratedBiais;
        double      propCalibratedScale;
        double      propDeadZone;

        //Gyrometer position in radian in [-pi;pi[
        RTT::OutputPort<double> outVelocity;
        RTT::OutputPort<double> outVelocityDegree;
        RTT::OutputPort<double> outAngleEuler;
        RTT::OutputPort<double> outAngleEulerDegree;
        RTT::OutputPort<double> outAngleSimpson;
        RTT::OutputPort<double> outAngleSimpsonDegree;
        RTT::OutputPort<double> outRawData;
};

} /* namespace arp_stm32 */
#endif /* GYROMETER_HPP_ */
