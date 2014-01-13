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
    CanOpenNode(name),
    attrPdoIndex(-1)
{
	m_outputs = CanARDDictionnaryAccessor::getUNS8Pointer(name,"outputs");

	addAttribute("attrPdoIndex",attrPdoIndex);

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

void WoodheadOut::preopHook()
{
    //On ne cherche le numero de PDO que si on ne l'a jamais fait, sinon ça merde pendant le reset...
    if( attrPdoIndex == -1 )
    {
        //recuperation de l'index du PDO transmit dans la table CanFestival
       attrPdoIndex = CanARDDictionnaryAccessor::getTransmitPdoIndex(0x200 + propNodeId);
       //si on est là, on n'a pas trouvé.
       if( attrPdoIndex < 0)
       {
           LOG(Error) << "Failed to find WoodHeadOut Command PDO index among "
                   << CanARDDictionnaryAccessor::getTransmitPdoNumber()
           << " tested index. Check your dictionnary." << endlog();
           m_RunningState = UNCONNECTED;
           return;
       }
    }

   CanOpenNode::preopHook();
}

void WoodheadOut::operationalHook()
{
    //appel du parent car il log les bootUp
    CanOpenNode::operationalHook();

    UNS8 inputs = 0;
    bool tmpRead;

    tmpRead = false;
    inBit1.readNewest(tmpRead);
    inputs += tmpRead;

    tmpRead = false;
    inBit2.readNewest(tmpRead);
    inputs += tmpRead<<1;

    tmpRead = false;
    inBit3.readNewest(tmpRead);
    inputs += tmpRead<<2;

    tmpRead = false;
    inBit4.readNewest(tmpRead);
    inputs += tmpRead<<3;

    tmpRead = false;
    inBit5.readNewest(tmpRead);
    inputs += tmpRead<<4;

    tmpRead = false;
    inBit6.readNewest(tmpRead);
    inputs += tmpRead<<5;

    tmpRead = false;
    inBit7.readNewest(tmpRead);
    inputs += tmpRead<<6;

    tmpRead = false;
    inBit8.readNewest(tmpRead);
    inputs += tmpRead<<7;

    EnterMutex();
    *m_outputs = inputs;
    LeaveMutex();

    //notification pour envoit du PDO operationnel de commande
    CanARD_Data.PDO_status[attrPdoIndex].last_message.cob_id = 0;
}
