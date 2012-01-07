/*
 * WoodheadOut.cpp
 *
 *  Created on: 29 apr. 2011
 *      Author: wla
 */

#include "WoodheadOut.hpp"
#include <rtt/Component.hpp>
#include "orocos/can/dictionnary/CanARD.h"
#include "orocos/can/wrappers/CanARDDictionnaryAccessor.hpp"

using namespace arp_hml;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_hml::WoodheadOut )

WoodheadOut::WoodheadOut(const std::string& name) :
    CanOpenNode(name)
{
	m_outputs = CanARDDictionnaryAccessor::getUNS8Pointer(name,"outputs");

    addPort("inBit1",inBit1)
        .doc("Value of the output #1");
    addPort("inBit2",inBit2)
        .doc("Value of the output #2");
    addPort("inBit3",inBit3)
        .doc("Value of the output #3");
    addPort("inBit4",inBit4)
        .doc("Value of the output #4");
    addPort("inBit5",inBit5)
        .doc("Value of the output #5");
    addPort("inBit6",inBit6)
        .doc("Value of the output #6");
    addPort("inBit7",inBit7)
        .doc("Value of the output #7");
    addPort("inBit8",inBit8)
        .doc("Value of the output #8");
}

bool WoodheadOut::checkInputsPorts()
{
	return true;
}

bool WoodheadOut::configureHook()
{
	bool res = CanOpenNode::configureHook();

	if( m_outputs == NULL )
	{
		res = false;
		LOG(Error) << "failed to configure : did not get CAN pointer m_outputs" << endlog();
	}
	else
	{
	    *m_outputs = 0;
	}

	return res;
}

void WoodheadOut::updateHook()
{
    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    UNS8 inputs = 0;
    bool tmpRead;

    inBit1.readNewest(tmpRead);
    inputs += tmpRead;
    inBit2.readNewest(tmpRead);
    inputs += tmpRead<<1;
    inBit3.readNewest(tmpRead);
    inputs += tmpRead<<2;
    inBit4.readNewest(tmpRead);
    inputs += tmpRead<<3;
    inBit5.readNewest(tmpRead);
    inputs += tmpRead<<4;
    inBit6.readNewest(tmpRead);
    inputs += tmpRead<<5;
    inBit7.readNewest(tmpRead);
    inputs += tmpRead<<6;
    inBit8.readNewest(tmpRead);
    inputs += tmpRead<<7;

    EnterMutex();
    *m_outputs = inputs;
    LeaveMutex();
}
