/*
 * CanopenNode.cpp
 *
 *  Created on: 21 mars 2011
 *      Author: wla
 */

#include "can/CanOpenNode.hpp"
#include <ocl/Component.hpp>
#include <rtt/scripting/ProgramInterface.hpp>

using namespace arp_hml;
using namespace arp_core;


CanOpenNode::CanOpenNode(const std::string& name):
	HmlTaskContext(name),
    propNodeId(int(0xFF)),
    propNmtTimeout(1000),
    propConfigureNodeTimeout(3000),
    propCanOpenControllerName("Can1"),
    inMasterClock(),
    inNmtState(),
    inBootUpFrame(),
    outRequestNmtState()
{
    updateNodeIdCard();

    addAttribute("attrCurrentNMTState",attrCurrentNMTState);


    addProperty("propNodeId",propNodeId)
        .doc("CAN adress of the node");
    addProperty("propNmtTimeout",propNmtTimeout)
        .doc("Timeout before considering a node is not responding to a NMT request (in ms)");
    addProperty("propConfigureNodeTimeout",propConfigureNodeTimeout)
        .doc("Timeout before considering configureNode has failed (in ms)");
    addProperty("propCanOpenControllerName",propCanOpenControllerName)
        .doc("name of the CanOpenController this component will connect");

    addEventPort("inMasterClock",inMasterClock)
    		.doc("");
    addPort("inNmtState",inNmtState)
            .doc("port from which we receive the NMT state of our node from a CanOpenController");
    addPort("inBootUpFrame",inBootUpFrame)
            .doc("port from which we receive the BootUp Frame of our node from a CanOpenController");
    addPort("outRequestNmtState",outRequestNmtState)
    		.doc("port in which the component can ask a new nmt state for our node from a CanOpenController");

    addOperation("coRegister", &CanOpenNode::coRegister, this, ClientThread)
    		.doc("use this operation in deployment to register the node into the CanOpenController");
}

void CanOpenNode::updateNodeIdCard()
{
    m_nodeIdCard.nodeId = propNodeId;
    m_nodeIdCard.task = this;
    m_nodeIdCard.inNmtState = &inNmtState;
    m_nodeIdCard.inBootUpFrame = &inBootUpFrame;
    m_nodeIdCard.outRequestNmtState = &outRequestNmtState;
}

bool CanOpenNode::coRegister()
{
	bool res = true;



    return res;
}

bool CanOpenNode::checkProperties()
{
    bool res = ARDTaskContext::checkProperties();

    if( propNodeId < 0 || propNodeId > 0xFF )
    {
        LOG(Error)  << "failed to check Properties : propNodeId is out of bounds [0x00;0xFF] : 0x" << std::hex << propNodeId << endlog();
        res = false;
    }

    if( propNodeId == 0x00 || propNodeId == 0x01 || propNodeId == 0xFF )
    {
        LOG(Warning)  << "propNodeId is a reserved value, it should not be used in operationnal : 0x" << std::hex << propNodeId << endlog();
    }

    if( propNmtTimeout <= 0 || propNmtTimeout > 2000 )
    {
        LOG(Error)  << "failed to check Properties : propNmtTimeout is out of bounds ]0;2000]: " << propNmtTimeout << endlog();
        res = false;
    }

    if( propConfigureNodeTimeout <= 0 || propConfigureNodeTimeout > 10000 )
    {
        LOG(Error)  << "failed to check Properties : propConfigureNodeTimeout is out of bounds ]0;10000]: " << propConfigureNodeTimeout << endlog();
        res = false;
    }

    if( propCanOpenControllerName == "" )
    {
        LOG(Error)  << "failed to check Properties : propCanOpenControllerName should not be empty." << endlog();
        res = false;
    }

    return res;
}

bool CanOpenNode::configureHook()
{
	TaskContext* tc = NULL;

    if( !ARDTaskContext::configureHook())
    	goto failed;

    //mise à jour de la nodeIdCard car la propriété a été lue dans le configureHook précédent
    updateNodeIdCard();

    //connexion aux opération de notre CanController favori
    if( !connectOperations() )
    	goto failed;

    //enregistrement du noeud dans le CanController associé
    if( m_ooRegister(m_nodeIdCard) == false )
    {
        LOG(Error)  << "coRegister failed : could not register into a CanOpenController" << endlog();
        goto failed;
    }

    //connect the NMT request port
    tc = getPeer(propCanOpenControllerName);
    if( tc == NULL )
    {
        LOG(Error) << "failed to configure : Controller TaskContext not found (is it a peer ?)" << endlog();
        goto failedUnregister;
    }
    else if( !inMasterClock.connectTo(tc->getPort("outNodesClock")) )
    {
        LOG(Error) << "failed to configure : inMasterClock failed to connect to outNodesClock" << endlog();
        goto failedUnregister;
    }

    //test des ports d'entrée
    if( inNmtState.connected()==false )
    {
        LOG(Error)  << "failed to configure : input port not connected : inNmtState" << endlog();
        goto failedUnregister;
    }
    if( inBootUpFrame.connected()==false )
    {
        LOG(Error)  << "failed to configure : input port not connected : inBootUpFrame" << endlog();
        goto failedUnregister;
    }


    //on envoie un reset au node pour être sure de partir sur de bonnes bases
    if( !resetNode() )
    {
        LOG(Error)  << "failed to configure : impossible to reset the node" << endlog();
    	goto failedUnregister;
    }

    //on envoie les SDO de configuration
    if( !configureNode() )
    {
        LOG(Error)  << "failed to configure : CAN configuration  failed" << endlog();
    	goto failedUnregister;
    }

    LOG(Info) << "CanOpenNode::configureHook : done" << endlog();
    goto success;

    success:
    	return true;
	failedUnregister:
		m_ooUnregister(propNodeId);
    	return false;
	failed:
    	return false;
}

bool CanOpenNode::startHook()
{
    bool res = ARDTaskContext::startHook();
    int chrono = 0;

    //envoit de la requête de reset au noeud
    outRequestNmtState.write(StartNode);
    //mise à jour de l'état NMT
    inNmtState.readNewest(attrCurrentNMTState);
    while( attrCurrentNMTState != Operational && chrono < propNmtTimeout )
    {
    	LOG(Debug) << "coMasterAskNmtNodeState : Node " << propNodeId << " is waiting for a NMT start frame ..." << endlog();
        chrono++;
        usleep(1000);
        inNmtState.readNewest(attrCurrentNMTState);
    }
    //si le timeout est explosé c'est que ça a foiré
    if( chrono >= propNmtTimeout )
    {
        LOG(Error)  << "startHook : timeout has expired, impossible to get into operational NMT state for node 0x"<< std::hex << propNodeId << endlog();
        goto failed;
    }

    LOG(Info) << "CanOpenNode::startHook : done" << endlog();
    return true;

    failed:
    return res;
}



void CanOpenNode::updateHook()
{
    ARDTaskContext::updateHook();

    //mise à jour de l'état NMT
    inNmtState.readNewest(attrCurrentNMTState);

    //lecture des bootup
    bool dummy;
    while( inBootUpFrame.read(dummy) == NewData )
    {
        ooReset();
        LOG(Info) << "Node " << propNodeId << " has send a boot up frame." << endlog();
    }

    //si l'état NMT n'est pas operationnel on peut arreter le composant
    if( attrCurrentNMTState != Operational )
    {
        LOG(Info) << "Node " << propNodeId << " has switch out of Operationnal mode" << endlog();
        stop();
    }
}

void CanOpenNode::stopHook()
{
    int chrono = 0;

    //envoit de la requête de reset au noeud
    outRequestNmtState.write(StopNode);
    //mise à jour de l'état NMT
    inNmtState.readNewest(attrCurrentNMTState);
    while( attrCurrentNMTState != Stopped && chrono < propNmtTimeout )
    {
    	LOG(Debug) << "stopHook : Node " << propNodeId << " is waiting for the StopNode request to be accepted ..." << endlog();
        chrono++;
        usleep(1000);
        inNmtState.readNewest(attrCurrentNMTState);
    }
    //si le timeout est explosé c'est que ça a foiré
    if( chrono >= propNmtTimeout )
    {
    	LOG(Fatal)  << "stopHook : Nmt StopNode request has not been processed by node 0x"<< std::hex << propNodeId << " stalled in state : " << attrCurrentNMTState << endlog();
        goto failed;
    }

    ARDTaskContext::stopHook();
    LOG(Info) << "CanOpenNode::stopHook : done" << endlog();

    failed:
    return;
}

void CanOpenNode::cleanupHook()
{
    m_ooUnregister(propNodeId);
    ARDTaskContext::cleanupHook();
    LOG(Info) << "CanOpenNode::cleanupHook : done" << endlog();
}

bool CanOpenNode::connectOperations()
{
    bool res = true;

    res &= getOperation(propCanOpenControllerName, "ooRegisterNewNode",       m_ooRegister);
    res &= getOperation(propCanOpenControllerName, "ooUnregisterNode",       m_ooUnregister);
    res &= getOperation(propCanOpenControllerName, "coWriteInRemoteDico",     m_coWriteInRemoteDico);
    res &= getOperation(propCanOpenControllerName, "coReadInRemoteDico",      m_coReadInRemoteDico);

    return res;
}

bool CanOpenNode::resetNode()
{
    bool res = true;
    bool bootUp;
    int chrono = 0;

    LOG(Info) << "Sending a reset to node 0x" << std::hex << propNodeId << endlog();

    //vidange de la pile de bootUp
    while( inBootUpFrame.read(bootUp) == NewData )
    {
        LOG(Warning)  << "resetNode : bootUp frame are pending, inBootUpFrame should be empty" << endlog();
    };

    //envoit de la requête de reset au noeud
    outRequestNmtState.write(ResetNode);

    //attente du bootup
    while ( inBootUpFrame.read(bootUp) != NewData && chrono < propNmtTimeout)
    {
        chrono += 50;
        usleep(1000*50);
    }
    if( chrono >= propNmtTimeout )
    {
        LOG(Error) << "resetNode : Node 0x" << std::hex << propNodeId << " is waiting for a bootUp frame ..." << endlog();
        res &= false;
        goto failed;
    }

    //mise à jour de l'état NMT
    while ( attrCurrentNMTState != Pre_operational && chrono < propNmtTimeout)
    {
        chrono += 50;
        inNmtState.readNewest(attrCurrentNMTState);
        usleep(1000*50);
    }
    if( chrono >= propNmtTimeout )
    {
        LOG(Error)  << "resetNode : timeout has expired, NMT state dispatch don't seem to be ok." << endlog();
        res &= false;
        goto failed;
    }

    if( res )
        LOG(Info) << "resetNode : success ! "  << endlog();

    return res;

    failed:
        return false;
}

bool CanOpenNode::configureNode()
{
    // start a program :
    if( scripting->hasProgram("configureNode") == false )
    {
        LOG(Error)  << "configureNode : did not found program configureNode ." << endlog();
        goto failed;
    }

    // start a program :
    if( scripting->startProgram("configureNode") == false )
    {
        LOG(Error)  << "configureNode : failed to start program configureNode." << endlog();
        goto failed;
    }

    //TODO WLA : tester si le programme est allé au bout
    return true;

    failed:
    return false;
}

