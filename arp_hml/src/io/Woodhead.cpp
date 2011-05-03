/*
 * Woodhead.cpp
 *
 *  Created on: 29 apr. 2011
 *      Author: wla
 */

#include "Woodhead.hpp"
#include <ocl/Component.hpp>
#include "can/dictionnary/CanARD.h"
#include "can/wrappers/CanARDDictionnaryAccessor.hpp"

using namespace arp_hml;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_hml::Woodhead )

Woodhead::Woodhead(const std::string& name) :
    CanOpenNode(name)
{
	m_outputs = CanARDDictionnaryAccessor::getUNS8Pointer("Woodhead","outputs");

    addPort("outBit1",outBit1)
        .doc("Value of the output #1");
    addPort("outBit2",outBit2)
        .doc("Value of the output #2");
    addPort("outBit3",outBit3)
        .doc("Value of the output #3");
    addPort("outBit4",outBit4)
        .doc("Value of the output #4");
    addPort("outBit5",outBit5)
        .doc("Value of the output #5");
    addPort("outBit6",outBit6)
        .doc("Value of the output #6");
    addPort("outBit7",outBit7)
        .doc("Value of the output #7");
    addPort("outBit8",outBit8)
        .doc("Value of the output #8");
}

bool Woodhead::configureHook()
{
	bool res = CanOpenNode::configureHook();

	if( m_outputs == NULL )
	{
		res = false;
		LOG(Error) << "failed to configure : did not get CAN pointer m_outputs" << endlog();
	}

	return res;
}

void Woodhead::updateHook()
{
    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    EnterMutex();
    UNS8 outputs = *m_outputs;
    LeaveMutex();

    outBit1.write(outputs & 0b00000001);
    outBit2.write(outputs & 0b00000010);
    outBit3.write(outputs & 0b00000100);
    outBit4.write(outputs & 0b00001000);
    outBit5.write(outputs & 0b00010000);
    outBit6.write(outputs & 0b00100000);
    outBit7.write(outputs & 0b01000000);
    outBit8.write(outputs & 0b10000000);

}
