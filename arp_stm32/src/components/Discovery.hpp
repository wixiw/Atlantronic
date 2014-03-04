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

class DiscoveryLock
{
    public:
        DiscoveryLock(pthread_mutex_t* mutex);
        ~DiscoveryLock();

       enum eLockResult
       {
           SUCCEED = 0,
           FAILED = 1
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
