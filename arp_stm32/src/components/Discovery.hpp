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

namespace arp_stm32
{

class Discovery: public Stm32TaskContext
{
    public:
        Discovery(const std::string& name);

        bool configureHook();
        void updateHook();
        void cleanupHook();

        /**
         * Ask the gyrometer to freeze its position and to start the calibration process
         */
        void ooStartCalibration();

        /**
         * Ask the gyrometer to stop the calibration process and to re-publish position datas
         */
        void ooStopCalibration();

        /**
         * Force a new gyrometer position, in rad
         */
        void ooSetPosition(double newAngle);

        /**
         * Resets the Stm32 board
         */
        void ooReset();

    protected:
        static void robotItfCallbackWrapper(void* arg);
        void robotItfUpdated();
        RobotInterface m_robotItf;
        void createOrocosInterface();
        double attrGyrometerAngle;

        /**
         * Orocos Interface
         */

        //Gyrometer position in radian in [-pi;pi[
        RTT::OutputPort<double> outGyrometerAngle;
        //unprocessed datas from the gyrometer
        RTT::OutputPort<double> outGyrometerRawData;

};

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
