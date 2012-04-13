/*
 * MasterTaskContext.cpp
 *
 *  Created on: 13 April 2012
 *      Author: wla
 */

#include "orocos/MasterTaskContext.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_master;

MasterTaskContext::MasterTaskContext(const std::string& name):
    ARDTaskContext(name, ros::package::getPath("arp_master") )
{

}

