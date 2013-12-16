/*
 * OdsTypeKit.cpp
 *
 *  Created on: 6 mars 2011
 *      Author: ard
 */

#include <rtt/types/TemplateConstructor.hpp>

#include "OdsTypeKit.hpp"

using namespace RTT;
using namespace arp_ods;

std::string OdsTypeKit::getName()
{
    return "OdsTypeKit";
}

bool OdsTypeKit::loadTypes()
{
    bool res = true;

    return res;
}

/** ...Add the other example code of this manual here as well... */
bool OdsTypeKit::loadConstructors()
{
    bool res = true;

    return res;
}

bool OdsTypeKit::loadOperators()
{
  // ...
    return true;
}


/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( arp_ods::OdsTypeKit );
