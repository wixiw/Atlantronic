/*
 * CanopenNode.cpp
 *
 *  Created on: 21 mars 2011
 *      Author: wla
 */

#include <rtt/Component.hpp>

#include "orocos/can/CanOpenNode.hpp"
#include <rtt/scripting/ProgramInterface.hpp>
#include <errno.h>

using namespace arp_hml;
using namespace arp_core;
using namespace scripting;
using namespace std;

CanOpenNode::CanOpenNode(const std::string& name):
	HmlTaskContext(name),
    propNodeId(int(0xFF)),
    propNmtTimeout(1.000),
    propCanOpenControllerName("Can1"),
    propCanConfigurationScript(""),
    propResetFirst(false),
    inMasterClock(),
    inBootUpFrame(),
    m_RunningState(arp_hml::UNKNOWN)
{
    updateNodeIdCard();

//    addAttribute("attrSyncTime", attrSyncTime);
    addAttribute("attrPeriod",attrPeriod);

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

    addPort("inMasterClock",inMasterClock)
    		.doc("");
    addPort("inBootUpFrame",inBootUpFrame)
            .doc("port from which we receive the BootUp Frame of our node from a CanOpenController");
    addPort("outOperationnalState",outOperationnalState)
            .doc("This port publish the internal sub states when running.");

    addOperation("ooEnterPreOp", &CanOpenNode::ooEnterPreOp, this, OwnThread)
            .doc("Switch to sub running state : PREOP");

    addOperation("coRegister", &CanOpenNode::coRegister, this, ClientThread)
    		.doc("use this operation in deployment to register the node into the CanOpenController");

    addOperation("coSendPdo", &CanOpenNode::coSendPdo,
            this, ClientThread) .doc("This operation allows anyone in the application to send a PDO. The PDO is automatically built from mapping information (you are so supposed to have updated the dico first).").arg(
            "pdoNumber","Number of the PDO in the dictionnay");

    addOperation("coRequestNmtChange", &CanOpenNode::coRequestNmtChange,
            this, ClientThread) .doc("This operation allows anyone in the application to request a new slave node NMT state. Take care it is a blocking operation, don't use in operationnal");

    outOperationnalState.write(m_RunningState);
}

void CanOpenNode::updateNodeIdCard()
{
    m_nodeIdCard.nodeId = propNodeId;
    m_nodeIdCard.task = this;
    m_nodeIdCard.inBootUpFrame = &inBootUpFrame;
    m_nodeIdCard.outRunningState = &outOperationnalState;
}

void CanOpenNode::updateLate()
{
    if( isRunning() && m_RunningState == arp_hml::OPERATIONAL)
    {
        updateLateHook();
    }
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
    else
    {
        if( !inMasterClock.connectTo(tc->getPort("outClock")) )
        {
            LOG(Error) << "failed to configure : inMasterClock failed to connect to outClock" << endlog();
            goto failedUnregister;
        }

        if( !inMasterPeriod.connectTo(tc->getPort("outPeriod")) )
        {
            LOG(Error) << "failed to configure : inMasterPeriod failed to connect to outPeriod" << endlog();
            goto failedUnregister;
        }
    }

    if( inBootUpFrame.connected()==false )
    {
        LOG(Error)  << "failed to configure : input port not connected : inBootUpFrame" << endlog();
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
    m_RunningState = arp_hml::UNCONNECTED;
    outOperationnalState.write(m_RunningState);
    LOG(Info) << "CanOpenNode::startingHook : done" << endlog();
    return res;
}


void CanOpenNode::updateHook()
{
    //Récupération de la date du cycle CAN
    inMasterClock.readNewest(attrSyncTime);
    inMasterPeriod.readNewest(attrPeriod);

    HmlTaskContext::updateHook();

    switch (m_RunningState)
    {
        case arp_hml::PREOP:
            preopHook();
            break;

        case arp_hml::OPERATIONAL:
            operationalHook();
            break;

        case arp_hml::IDLE:
            idleHook();
            break;

        case arp_hml::UNCONNECTED:
            unconnectedHook();
            break;

        default:
            LOG(Error) << "CanOpenController::updateHook(): unknown m_RunningState=" << m_RunningState << endlog();
            break;
    }

    outOperationnalState.write(m_RunningState);
}

void CanOpenNode::preopHook()
{
    LOG(Info)  << "CanOpenNode::preopHook : entering" << endlog();

    //normalement tous les noeuds sont sensés être reset lors du démarrage du controlleur.
    //en cas de doute, besoin de sécurité, de device ayant déjà réagit, ... il est possible de forcer un nouveau reset
    if( propResetFirst )
    {
        LOG(Info)  << "CanOpenNode::preopHook : reset forced" << endlog();

        //envoit de la requête de reset au noeud
        if( coRequestNmtChange(ResetNode) == false )
        {
            LOG(Error) << "CanOpenNode::preopHook : Return to UNCONNECTED state." << endlog();
            m_RunningState = arp_hml::UNCONNECTED;
            outOperationnalState.write(m_RunningState);
            return;
        }

        LOG(Info)  << "CanOpenNode::preopHook : reseted" << endlog();
    }

    //on envoie les SDO de configuration
    if( !configureNode() )
    {
        LOG(Error)  << "preopHook : Node 0x" << std::hex << propNodeId << " failed to configure. Return to UNCONNECTED state" << endlog();
        m_RunningState = arp_hml::UNCONNECTED;
        outOperationnalState.write(m_RunningState);
        return;
    }
    LOG(Info)  << "CanOpenNode::preopHook : configured" << endlog();

    //envoit de la requête de start au noeud
    if( coRequestNmtChange(StartNode) == false )
    {
        LOG(Error)  << "preopHook : timeout has expired, impossible to get into operational NMT state for node 0x"<< std::hex << propNodeId << ". Returning to UNCONNECTED state"<< endlog();
        m_RunningState = arp_hml::UNCONNECTED;
        outOperationnalState.write(m_RunningState);
        return;
    }

    //everything is ok
    m_RunningState = arp_hml::OPERATIONAL;
    outOperationnalState.write(m_RunningState);
    LOG(Info)  << "CanOpenNode::preopHook : going to Operationnal" << endlog();
}

void CanOpenNode::operationalHook()
{
    //lecture des bootup
    bool dummy;
    if( inBootUpFrame.read(dummy) == NewData )
    {
        m_RunningState = arp_hml::IDLE;
        outOperationnalState.write(m_RunningState);
        LOG(Info) << "operationalHook: Node " << propNodeId << " has send a boot up frame. Returning to IDLE" << endlog();
    }
}

void CanOpenNode::idleHook()
{
}

void CanOpenNode::unconnectedHook()
{
    //lecture des bootup
    bool dummy;
    if( inBootUpFrame.read(dummy) == NewData )
    {
        m_RunningState = arp_hml::IDLE;
        outOperationnalState.write(m_RunningState);
        LOG(Info) << "unconnectedHook: Node " << propNodeId << " has send a boot up frame. Returning to IDLE" << endlog();
    }
}

void CanOpenNode::stopHook()
{
    //envoit de la requête de stop au noeud
    if( coRequestNmtChange(StopNode) == false )
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
    res &= getOperation(propCanOpenControllerName, "coRequestNmtChange",        m_coRequestNmtChange);


    return res;
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

bool CanOpenNode::ooEnterPreOp()
{
    if( !isRunning() || m_RunningState != arp_hml::IDLE )
    {
        LOG(Error)  << "ooEnterPreOp : you are trying to switch to PreOp state but current state doesn't allow this (sub state=" << m_RunningState << ")." << endlog();
        return false;
    }

    m_RunningState = arp_hml::PREOP;
    outOperationnalState.write(m_RunningState);
    LOG(Info)  << "ooEnterPreOp : go to PreOp." << endlog();

    update();

    return true;
}

bool CanOpenNode::coSendPdo(int pdoNumber)
{
    Message pdo;
    memset(&pdo, 0, sizeof(pdo));
    bool res = false;
    
    EnterMutex();
    if (buildPDO (&CanARD_Data, pdoNumber, &pdo))
    {
        LeaveMutex();
        LOG(Error) << "coSendPdo: build PDO " << pdoNumber << " failed" << endlog();
        return false;
    }
    CanARD_Data.PDO_status[pdoNumber].last_message = pdo;
    res = canSend (CanARD_Data.canHandle, &pdo);
    if( res )
    {
        LeaveMutex();
        LOG(Error) << "coSendPdo: failed to send PDO 0x" << std::hex << pdo.cob_id << " "
                << pdo.data[0] << "."
                << pdo.data[1] << "."
                << pdo.data[2] << "."
                << pdo.data[3] << "."
                << pdo.data[4] << "."
                << pdo.data[5] << "."
                << pdo.data[6] << "."
                << pdo.data[7] << "."
                << std::dec << endlog();
        return false;
    }

    LeaveMutex();
    return true;
}


bool CanOpenNode::coRequestNmtChange(enum_DS301_nmtStateRequest nmtStateCmd)
{
    bool bootUp;

    //s'il s'agit d'un reset on vide le port d'ecoute du bootup
    if( nmtStateCmd == NMT_Reset_Node ||  nmtStateCmd == NMT_Reset_Comunication )
    {
        //vidange de la pile de bootUp
        while( inBootUpFrame.read(bootUp) == NewData )
        {
            LOG(Warning)  << "CanOpenController::coRequestNmtChange : bootUp frame are pending, inBootUpFrame should be empty" << endlog();
        };
    }


    //send NMT cmd
    if( !m_coRequestNmtChange(propNodeId,nmtStateCmd,propNmtTimeout) )
    {
        LOG(Error) << "CanOpenController::coRequestNmtChange : failed to send NMT command." << endlog();
        goto failed;
    }

    goto success;

    failed:
        return false;
    success:
        return true;
}
