/*
 * CoreTypeKit.cpp
 *
 *  Created on: 02 April 2011
 *      Author: ard
 */

#include <rtt/types/TemplateConstructor.hpp>

#include "CoreTypeKit.hpp"
#include "Pose2DTypeInfo.hpp"


using namespace RTT;
using namespace arp_core;

std::string CoreTypeKit::getName()
{
    return "CoreTypeKit";
}

bool CoreTypeKit::loadTypes()
{
    bool res = true;

    // Tell the RTT the name and type of this struct
    res &= types::Types()->addType( new Pose2DTypeInfo() );

    return res;
}

/** ...Add the other example code of this manual here as well... */
bool CoreTypeKit::loadConstructors()
{
    bool res = true;

    types::Types()->type("Pose2D")->addConstructor( types::newConstructor(&createPose2D) );

    return res;
}

bool CoreTypeKit::loadOperators()
{
  // ...
    return true;
}


/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( arp_core::CoreTypeKit );
