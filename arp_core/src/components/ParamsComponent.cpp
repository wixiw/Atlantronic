/*
 * ParamsComponent.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "ParamsComponent.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_core;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_core::ParamsComponent )

ParamsComponent::ParamsComponent(const std::string& name):
        ARDTaskContext(name,ros::package::getPath("arp_core"))
{
    addProperty("propSavedParams",propSavedParams);

    addPort("outParams",outParams);
}

bool ParamsComponent::configureHook()
{
    if( !ARDTaskContext::configureHook() )
    {
        LOG(Error) << "ARDTaskContext failed to configure" << endlog();
        goto fail;
    }

    if( ! propSavedParams.check() )
    {
        LOG(Error) << "Loading parameters are not consistent." << endlog();
        goto fail;
    }

    goto success;

    fail:
        return false;
    success:
        return true;
}


void ParamsComponent::updateHook()
{
    outParams.write(propSavedParams);
}