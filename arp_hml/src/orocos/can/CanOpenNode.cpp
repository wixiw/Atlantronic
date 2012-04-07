/*
 * CanopenNode.cpp
 *
 *  Created on: 21 mars 2011
 *      Author: wla
 */

#include <rtt/Component.hpp>

#include "orocos/can/CanOpenNode.hpp"
#include <rtt/scripting/ProgramInterface.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace scripting;

CanOpenNode::CanOpenNode(const std::string& name):
	HmlTaskContext(name),
    propNodeId(int(0xFF)),
    propNmtTimeout(1.000),
    propCanOpenControllerName("Can1"),
    propCanConfigurationScript(""),
    propResetFirst(true),
    inMasterClock(),
    inBootUpFrame()
{
    updateNodeIdCard();

    addAttribute("attrSyncTime", attrSyncTime);

    addProperty("propNodeId",propNodeId)
        .doc("CAN adress of the node");
    addProperty("propNmtTimeout",propNmtTimeout)
        .doc("Timeout before considering a node is not responding to a NMT request (in s)");
    addProperty("propCanOpenControllerName",propCanOpenControllerName)
        .doc("name of the CanOpenController this component will connect");
    addProperty("propCanConfigurationScript",propCanConfigurationScript)
        .doc("this script is executed in the configureHook to set up CAN config values");
    addProperty("propResetFirst",propResetFirst)
        .doc("Require a CAN node reset during configure. As the controller is already doing it, it is usually not usefull");

    addEventPort("inMasterClock",inMasterClock)
    		.doc("");
    addPort("inBootUpFrame",inBootUpFrame)
            .doc("port from which we receive the BootUp Frame of our node from a CanOpenController");
    addPort("outConnected",outConnected)
            .doc("This port is true when the component thinks the device is disconnected of the network");

    addOperation("coRegister", &CanOpenNode::coRegister, this, ClientThread)
    		.doc("use this operation in deployment to register the node into the CanOpenController");

    outConnected.write(false);
}

void CanOpenNode::updateNodeIdCard()
{
    m_nodeIdCard.nodeId = propNodeId;
    m_nodeIdCard.task = this;
    m_nodeIdCard.inBootUpFrame = &inBootUpFrame;
}

bool CanOpenNode::coRegister()
{
	bool res = true;



    return res;
}

bool CanOpenNode::checkProperties()
{
    bool res = HmlTaskContext::checkProperties();

    if( propNodeId < 0 || propNodeId > 0xFF )
    {
        LOG(Error)  << "failed to check Properties : propNodeId is out of bounds [0x00;0xFF] : 0x" << std::hex << propNodeId << endlog();
        res = false;
    }

    if( propNodeId == 0x00 || propNodeId == 0x01 || propNodeId == 0xFF )
    {
        LOG(Warning)  << "propNodeId is a reserved value, it should not be used in operationnal : 0x" << std::hex << propNodeId << endlog();
    }

    if( propNmtTimeout <= 0 || propNmtTimeout > 20 )
    {
        LOG(Error)  << "failed to check Properties : propNmtTimeout is out of bounds ]0;20]: " << propNmtTimeout << endlog();
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

    if( !HmlTaskContext::configureHook())
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

    if( inBootUpFrame.connected()==false )
    {
        LOG(Error)  << "failed to configure : input port not connected : inBootUpFrame" << endlog();
        goto failedUnregister;
    }

    //normalement tous les noeuds sont sensés être reset lors du démarrage du controlleur.
    //en cas de doute, besoin de sécurité, de device ayant déjà réagit, ... il est possible de forcer un nouveau reset
    if( propResetFirst )
    {
        if( !resetNode() )
        {
            LOG(Error)  << "failed to configure : impossible to reset the node" << endlog();
            goto failedUnregister;
        }
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
    bool res = HmlTaskContext::startHook();
    if ( res == false )
        goto failed;

    //envoit de la requête de start au noeud
    if( m_coSendNmtCmd(propNodeId, StartNode, propNmtTimeout) == false )
    {
        LOG(Error)  << "startHook : timeout has expired, impossible to get into operational NMT state for node 0x"<< std::hex << propNodeId << endlog();
        goto failed;
    }

    LOG(Info) << "CanOpenNode::startHook : done" << endlog();
    outConnected.write(true);
    return true;

    failed:
    return false;
}



void CanOpenNode::updateHook()
{
    //Récupération de la date du cycle CAN
    inMasterClock.readNewest(attrSyncTime);

    HmlTaskContext::updateHook();

    //lecture des bootup
    bool dummy;
    if( inBootUpFrame.read(dummy) == NewData )
    {
        stop();
        LOG(Info) << "Node " << propNodeId << " has send a boot up frame." << endlog();
    }
}

void CanOpenNode::stopHook()
{
    //envoit de la requête de stop au noeud
    if( m_coSendNmtCmd(propNodeId, StopNode, propNmtTimeout) == false )
    {
        LOG(Error)  << "stopHook : timeout has expired, impossible to get into operational NMT state for node 0x"<< std::hex << propNodeId << endlog();
        goto failed;
    }


    HmlTaskContext::stopHook();
    LOG(Info) << "CanOpenNode::stopHook : done" << endlog();
    goto success;

    failed:
        return;
    success:
        return;
}

void CanOpenNode::cleanupHook()
{
    m_ooUnregister(propNodeId);
    HmlTaskContext::cleanupHook();
    LOG(Info) << "CanOpenNode::cleanupHook : done" << endlog();
}

bool CanOpenNode::connectOperations()
{
    bool res = true;

    res &= getOperation(propCanOpenControllerName, "ooRegisterNewNode",         m_ooRegister);
    res &= getOperation(propCanOpenControllerName, "ooUnregisterNode",          m_ooUnregister);
    res &= getOperation(propCanOpenControllerName, "coWriteInRemoteDico",       m_coWriteInRemoteDico);
    res &= getOperation(propCanOpenControllerName, "coReadInRemoteDico",        m_coReadInRemoteDico);
    res &= getOperation(propCanOpenControllerName, "coSendPdo",                 m_coSendPdo);
    res &= getOperation(propCanOpenControllerName, "coSendNmtCmd",              m_coSendNmtCmd);

    return res;
}

bool CanOpenNode::resetNode()
{
    bool bootUp;
    double chrono = 0.0;

    LOG(Info) << "Sending a CAN reset to node 0x" << std::hex << propNodeId << endlog();

    //vidange de la pile de bootUp
    while( inBootUpFrame.read(bootUp) == NewData )
    {
        LOG(Warning)  << "resetNode : bootUp frame are pending, inBootUpFrame should be empty" << endlog();
    };

    //envoit de la requête de reset au noeud
    if( m_coSendNmtCmd(propNodeId, ResetNode, propNmtTimeout) == false )
    {
        LOG(Error) << "resetNode : Node 0x" << std::hex << propNodeId << " failed to send the NMT change request" << endlog();
        goto failed;
    }

    //attente du bootup
    whileTimeout ( inBootUpFrame.read(bootUp) != NewData , propNmtTimeout, 0.050);
    IfWhileTimeoutExpired(propNmtTimeout)
    {
        LOG(Error) << "resetNode : Node 0x" << std::hex << propNodeId << " is waiting for a bootUp frame ..." << endlog();
        goto failed;
    }

    LOG(Info) << "resetNode : success ! "  << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool CanOpenNode::configureNode()
{
	//initialisation of success flag
	attrScriptRes = true;

    // start a program :
	if( propCanConfigurationScript == "" )
	{
	    LOG(Warning)  << "configureNode : no script is define for CAN configuration" << endlog();
	    goto success;
	}
	else
	{
        if( scripting->runScript( attrProjectRootPath + "/" + attrScriptPath + "/can/" + propCanConfigurationScript) == false )
        {
            LOG(Error)  << "configureNode : failed to execute CAN script : " << propCanConfigurationScript << endlog();
            goto failed;
        }
	}

    // check the result :
    if( attrScriptRes == true )
    {
        LOG(Info)  << "configureNode : program finished properly." << endlog();
        goto success;
    }
    else
    {
        LOG(Error)  << "configureNode : failed to reached program end." << endlog();
        goto failed;
    }



    failed:
    	return false;
    success:
    	return true;
}

