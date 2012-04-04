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
        goto fail;
    }

    if( ! propSavedParams.check() )
    {
        goto fail;
        LOG(Error) << "Loading parameters are not consistent." << endlog();
    }

    outParams.write(propSavedParams);
    goto success;

    fail:
        return false;
    success:
        return true;
}
