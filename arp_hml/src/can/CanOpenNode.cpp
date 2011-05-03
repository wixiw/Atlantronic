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
    ARDTaskContext(name),
    propNodeId(int(0xFF)),
    propNmtTimeout(1000),
    propConfigureNodeTimeout(3000),
    propCanOpenControllerName("Can1"),
    inNmtState(),
    inBootUpFrame()
{
    //TODO WLA : workaround en attendant de trouver dans quel dossier on est lancé dans ROS
    attrPropertyPath = "/opt/ros/ard/arp_hml/script/orocos/conf";
    attrScriptPath = "/opt/ros/ard/arp_hml/script/orocos/ops";
    attrStateMachinePath = "/opt/ros/ard/arp_hml/script/orocos/osd";

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

    addPort("inNmtState",inNmtState)
            .doc("port from which we receive the NMT state of our node from a CanOpenController");
    addPort("inBootUpFrame",inBootUpFrame)
            .doc("port from which we receive the BootUp Frame of our node from a CanOpenController");
}

void CanOpenNode::updateNodeIdCard()
{
    m_nodeIdCard.nodeId = propNodeId;
    m_nodeIdCard.task = this;
    m_nodeIdCard.inNmtState = &inNmtState;
    m_nodeIdCard.inBootUpFrame = &inBootUpFrame;
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
    bool res = ARDTaskContext::configureHook();

    //mise à jour de la nodeIdCard car la propriété a été lue dans le configureHook précédent
    updateNodeIdCard();

    //connexion aux opération de notre CanController favori
    res &= connectOperations();

    //enregistrement du noeud dans le CanController associé
    if( m_ooRegister(m_nodeIdCard) == false )
    {
        res = false;
        LOG(Error)  << "failed to configure : could not register into a CanOpenController" << endlog();
    }

    //on envoie un reset au node pour être sure de partir sur de bonnes bases
    res &= resetNode();

    //on envoie les SDO de configuration
    res &= configureNode();

    if( res )
        LOG(Info) << "CanOpenNode::configureHook : done" << endlog();

    return res;
}

bool CanOpenNode::startHook()
{
    bool res = ARDTaskContext::startHook();
    e_nodeState nmtState = Unknown_state;

    //envoit de la requête de reset au noeud
    nmtState = m_coMasterSetNmtNodeState(propNodeId, StartNode, propNmtTimeout );
    //mise à jour de l'état NMT
    inNmtState.readNewest(attrCurrentNMTState);

    //vérification du résultat
    if( nmtState != Operational && attrCurrentNMTState!= Operational )
    {
       res = false;
       LOG(Error)  << "startHook : Nmt StartNode request has not been processed by node 0x"<< std::hex << propNodeId << " stalled in state : " << nmtState << endlog();
       goto failed;
    }

    //TODO WLA mettre un timeout
    while ( attrCurrentNMTState != Operational )
        inNmtState.readNewest(attrCurrentNMTState);

    LOG(Info) << "CanOpenNode::startHook : done" << endlog();

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
    e_nodeState nmtState = Unknown_state;

    //envoit de la requête de reset au noeud
    nmtState = m_coMasterSetNmtNodeState(propNodeId, StopNode, propNmtTimeout );
    inNmtState.readNewest(attrCurrentNMTState);

    //vérification du résultat
    if( nmtState != Stopped || attrCurrentNMTState != Stopped )
    {
       LOG(Fatal)  << "stopHook : Nmt StopNode request has not been processed by node 0x"<< std::hex << propNodeId << " stalled in state : " << nmtState << endlog();
    }

    ARDTaskContext::stopHook();
    LOG(Info) << "CanOpenNode::stopHook : done" << endlog();
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
    res &= getOperation(propCanOpenControllerName, "coMasterSetNmtNodeState", m_coMasterSetNmtNodeState);
    res &= getOperation(propCanOpenControllerName, "coMasterAskNmtNodeState", m_coMasterAskNmtNodeState);

    return res;
}

bool CanOpenNode::resetNode()
{
    bool res = true;
    bool bootUp;
    enum_nodeState nmtState = Unknown_state;
    int chrono = 0;

    LOG(Info) << "Sending a reset to node 0x" << std::hex << propNodeId << endlog();

    //vidange de la pile de bootUp
    while( inBootUpFrame.read(bootUp) == NewData )
    {
        LOG(Warning)  << "resetNode : bootUp frame are pending, inBootUpFrame should be empty" << endlog();
    };

    //envoit de la requête de reset au noeud
    nmtState = m_coMasterSetNmtNodeState(propNodeId, ResetNode, 0);

    //attente du bootup
    while ( inBootUpFrame.read(bootUp) != NewData && chrono < propNmtTimeout)
    {
        chrono += 50;
        usleep(1000*50);
        m_coMasterAskNmtNodeState(propNodeId, 0);
    }
    if( chrono >= propNmtTimeout )
    {
        LOG(Error) << "resetNode : Node " << propNodeId << " is waiting for a bootUp frame ..." << endlog();
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
    bool res = true;

    // start a program :
    if( scripting->hasProgram("configureNode") == false )
    {
        LOG(Error)  << "configureNode : did not found program configureNode ." << endlog();
        res &= false;
        goto result;
    }

    // start a program :
    if( scripting->startProgram("configureNode") == false )
    {
        LOG(Error)  << "configureNode : failed to start program configureNode." << endlog();
        res &= false;
        goto result;
    }

    //TODO WLA : tester si le programme est allé au bout

    result:
    return res;
}

