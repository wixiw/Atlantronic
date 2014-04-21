/*
 * MotionScheduler.hpp
 *
 *  Created on: Apr 27, 2012
 *      Author: ard
 */

#ifndef MOTIONSCHEDULER_HPP_
#define MOTIONSCHEDULER_HPP_

#include <fbsched/fbsched.hpp>
#include <time/StatTimer.hpp>
#include <time/ArdTime.hpp>

namespace arp_core
{

class MotionScheduler: public FBSched
{
    public:
        MotionScheduler(const std::string& name,const std::string& packageName);
        virtual ~MotionScheduler();

        /**
         * Charge le service de marshalling Orocos et charge les propriétés.
         */
        bool configureHook();

        /**
         * Reset timer for reporting
         */
        bool startHook();

        /**
         * Update time reporting
         */
        void updateHook();

        /**
         */
        void stopHook();

    protected:
        bool propTimeReporting;
        RTT::InputPort<arp_time::ArdAbsoluteTime> inClock;
        arp_core::StatTimer m_timer;
        std::string m_packageName;

        void ooGetPerformanceReport();
        void ooSetMaxBufferSize(unsigned int size);
};

} /* namespace arp_core */
#endif /* MOTIONSCHEDULER_HPP_ */
