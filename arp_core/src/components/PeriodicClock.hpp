/*
 * PeriodicClock.hpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#ifndef PERIODICCLOCK_HPP_
#define PERIODICCLOCK_HPP_

#include "taskcontexts/ARDTaskContext.hpp"

namespace arp_core
{

class PeriodicClock: public ARDTaskContext
{
    public:
        PeriodicClock(const std::string name);
        void updateHook();

    protected:
        OutputPort<double> outClock;
};

} /* namespace arp_core */
#endif /* PERIODICCLOCK_HPP_ */
