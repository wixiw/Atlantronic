/*
 * MotionControl.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef MOTIONCONTROL_HPP_
#define MOTIONCONTROL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"

namespace arp_ods
{

class KinematicBase: public OdsTaskContext
{
    public:
        KinematicBase(const std::string& name);

    protected:
        /**
         * Clock port which trigger our activity
         * It contains the time at which the input data are supposed to be calculated
         */
        InputPort<timespec> inClock;
};

} /* namespace arp_ods */
#endif /* MOTIONCONTROL_HPP_ */
