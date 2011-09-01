/*
 * HmlTaskContext.cpp
 *
 *  Created on: 04 may 2011
 *      Author: wla
 */

#include "HmlTaskContext.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_hml;
using namespace arp_core;

HmlTaskContext::HmlTaskContext(const std::string& name):
    ARDTaskContext(name, ros::package::getPath("arp_hml") )
{

}

