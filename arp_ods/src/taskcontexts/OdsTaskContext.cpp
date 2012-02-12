/*
 * OdsTaskContext.cpp
 *
 *  Created on: 12 février 2012
 *      Author: wla
 */

#include "OdsTaskContext.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_ods;
using namespace arp_core;

OdsTaskContext::OdsTaskContext(const std::string& name):
    ARDTaskContext(name, ros::package::getPath("arp_ods") )
{

}

