/*
 * WoodheadSimul.cpp
 *
 *  Created on: 29 apr. 2011
 *      Author: wla
 */

#include "WoodheadSimul.hpp"
#include <rtt/Component.hpp>

using namespace arp_hml;


ORO_LIST_COMPONENT_TYPE( arp_hml::WoodheadSimul )

WoodheadSimul::WoodheadSimul(const std::string& name):
    HmlTaskContext(name)
{
    addPort("outBit1",outBit1)
        .doc("Value of the input #1");
    addPort("outBit2",outBit2)
        .doc("Value of the input #2");
    addPort("outBit3",outBit3)
        .doc("Value of the input #3");
    addPort("outBit4",outBit4)
        .doc("Value of the input #4");
    addPort("outBit5",outBit5)
        .doc("Value of the input #5");
    addPort("outBit6",outBit6)
        .doc("Value of the input #6");
    addPort("outBit7",outBit7)
        .doc("Value of the input #7");
    addPort("outBit8",outBit8)
        .doc("Value of the input #8");

    addPort("outConnected",outConnected)
        .doc("Always true in Simul");

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

    outBit1.write(false);
    outBit2.write(false);
    outBit3.write(false);
    outBit4.write(false);
    outBit5.write(false);
    outBit6.write(false);
    outBit7.write(false);
    outBit8.write(false);

    outConnected.write(true);
}

void WoodheadSimul::updateHook()
{
    outBit1.write(false);
    outBit2.write(false);
    outBit3.write(false);
    outBit4.write(false);
    outBit5.write(false);
    outBit6.write(false);
    outBit7.write(false);
    outBit8.write(false);
}
