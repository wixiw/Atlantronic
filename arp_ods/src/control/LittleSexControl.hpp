/*
 * LittleSexControl.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef LITTLESEXCONTROL_HPP_
#define LITTLESEXCONTROL_HPP_

#include "taskcontexts/OdsTaskContext.hpp"

namespace arp_ods
{

class LittleSexControl: public OdsTaskContext
{
    public:
        LittleSexControl(const std::string& name);

    protected:
        /**
         * Clock port which trigger our activity
         * It contains the time at which the input data are supposed to be calculated
         */
        InputPort<timespec> inClock;
};

} /* namespace arp_ods */
#endif /* LITTLESEXCONTROL_HPP_ */
