/*
 * RluTaskContext.cpp
 *
 *  Created on: 5 Avril 2012
 *      Author: wla
 */

#include "RluTaskContext.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_rlu;
using namespace arp_core;

RluTaskContext::RluTaskContext(const std::string& name):
    ARDTaskContext(name, ros::package::getPath("arp_rlu") )
{

}

