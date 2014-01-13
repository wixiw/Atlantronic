/*
 * HMLTypeKit.cpp
 *
 *  Created on: 6 mars 2011
 *      Author: ard
 */

#include <rtt/types/TemplateConstructor.hpp>

#include "HMLTypeKit.hpp"
#include "CanStateTypeInfo.hpp"
#include "CanNodeRunningStateTypeInfo.hpp"
#include "CanStateRequestTypeInfo.hpp"
#include "DS402StateTypeInfo.hpp"
#include "CanDicoEntryTypeInfo.hpp"

using namespace RTT;
using namespace arp_hml;

std::string HMLTypeKit::getName()
{
    return "HMLTypeKit";
}

bool HMLTypeKit::loadTypes()
{
    bool res = true;

    // Tell the RTT the name and type of this struct
    res &= types::Types()->addType( new NMTStateTypeInfo() );
    res &= types::Types()->addType( new NMTStateRequestTypeInfo() );
    res &= types::Types()->addType( new DS402StateTypeInfo() );
    res &= types::Types()->addType( new CanDicoEntryTypeInfo() );
    res &= types::Types()->addType( new CanNodeRunningStateTypeInfo() );

    return res;
}

/** ...Add the other example code of this manual here as well... */
bool HMLTypeKit::loadConstructors()
{
    bool res = true;

    types::Types()->type("CanDicoEntry")->addConstructor( types::newConstructor(&createCanDicoEntry) );
    types::Types()->type("NMTStateRequest")->addConstructor( types::newConstructor(&createNmtStateRequest) );

    return res;
}

bool HMLTypeKit::loadOperators()
{
  // ...
    return true;
}


/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( arp_hml::HMLTypeKit );
