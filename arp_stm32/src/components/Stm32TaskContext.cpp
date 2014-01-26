/*
 * Stm32TaskContext.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Stm32TaskContext.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_stm32;
using namespace arp_core;

Stm32TaskContext::Stm32TaskContext(const std::string& name) :
        ARDTaskContext(name, ros::package::getPath("arp_stm32"))
{

}

