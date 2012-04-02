/*
 * KinematicBase.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "KinematicBase.hpp"
#include <rtt/Component.hpp>

using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::KinematicBase )

KinematicBase::KinematicBase(const std::string& name):
        OdsTaskContext(name),
        inClock()
{
    addEventPort("inClock",inClock)
            .doc("Clock port which trigger our activity. It contains the time at which the input data are supposed to be calculated");
}

