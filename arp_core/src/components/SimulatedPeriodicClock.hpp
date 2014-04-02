/*
 * SimulatedPeriodicClock.hpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#ifndef SIMULATEDPERIODICCLOCK_HPP_
#define SIMULATEDPERIODICCLOCK_HPP_

#include "taskcontexts/ARDTaskContext.hpp"
#include "time/ArdTime.hpp"

namespace arp_core
{

class SimulatedPeriodicClock: public ARDTaskContext
{
    public:
        SimulatedPeriodicClock(const std::string name);
        bool startHook();
        void updateHook();

    protected:
        RTT::OutputPort<arp_time::ArdAbsoluteTime> outClock;
        RTT::OutputPort<double> outClockReporting; //workaround as typekit is segfaulting with ArdAbsolutetime
        RTT::OutputPort<arp_time::ArdTimeDelta> outPeriod;
        RTT::OutputPort<int> outTrigger;

        RTT::os::TimeService* m_ts;
        arp_time::ArdAbsoluteTime m_lastTime;

};

} /* namespace arp_core */
#endif /* PERIODICCLOCK_HPP_ */
