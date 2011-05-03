/*
 * SimplePCM3362.cpp
 *
 *  Created on: 2 nov. 2010
 *      Author: ard
 */

#include "SimplePCM3362.hpp"
#include <ocl/Component.hpp>
#include <susi.h>

using namespace arp_hml;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_hml::SimplePCM3362 )

SimplePCM3362::SimplePCM3362(const std::string& name) :
    ARDTaskContext(name)
{
    //TODO WLA : workaround en attendant de trouver dans quel dossier on est lancé dans ROS
    attrPropertyPath = "/opt/ros/ard/arp_hml/script/orocos/conf";
    attrScriptPath = "/opt/ros/ard/arp_hml/script/orocos/ops";
    attrStateMachinePath = "/opt/ros/ard/arp_hml/script/orocos/osd";

//    if ( !SusiDllInit() )
//    {
//        LOG(Error) << "Susi initialisation failed. Error number " << SusiDllGetLastError()<< endlog();
//    }
}

SimplePCM3362::~SimplePCM3362()
{
//    if ( !SusiDllUnInit() )
//    {
//        LOG(Error) << "Susi un-initialisation failed" << endlog();
//    }
}

