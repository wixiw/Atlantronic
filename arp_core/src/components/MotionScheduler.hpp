/*
 * MotionScheduler.hpp
 *
 *  Created on: Apr 27, 2012
 *      Author: ard
 */

#ifndef MOTIONSCHEDULER_HPP_
#define MOTIONSCHEDULER_HPP_

#include <fbsched/fbsched.hpp>
#include <timer/StatTimer.hpp>

namespace arp_core
{

class MotionScheduler: public FBSched
{
    public:
        MotionScheduler(const std::string& name);
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
        double attrLastPeriod;
        double attrLastComputationTime;
        RTT::InputPort<timespec> inClock;
        arp_core::StatTimer m_timer;

        void ooGetPerformanceReport();
        void ooSetMaxBufferSize(unsigned int size);
};

} /* namespace arp_core */
#endif /* MOTIONSCHEDULER_HPP_ */
