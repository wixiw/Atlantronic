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

#include <arp_stm32/SetGyroPosition.h>
#include <arp_core/EmptyWithSuccess.h>

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
        bool srvStartCalibration(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooStopCalibration operation
         */
        bool srvStopCalibration(SetGyroPosition::Request& req, SetGyroPosition::Response& res);

        /**
         * ROS wrapper on the HmlMonitor.ooSetPosition operation
         */
        bool srvSetPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res);


    protected:
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

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

        std::vector<ros::ServiceServer> m_srvList;
};

} /* namespace arp_stm32 */
#endif /* GYROMETER_HPP_ */
