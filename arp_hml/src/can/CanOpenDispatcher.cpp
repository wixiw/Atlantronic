/*
 * CanOpenDispatcher.cpp
 *
 *  Created on: 28 mars 2011
 *      Author: wla
 */

#include "CanOpenDispatcher.hpp"
#include "can/dictionnary/CanARD.h"
#include <rtt/extras/SlaveActivity.hpp>

//TODO WLA workaround : impossible de logguer en dehors d'un composant ?
#define LOG(truc) cout

using namespace arp_hml;
using namespace RTT;
using namespace extras;

CanOpenDispatcher::CanOpenDispatcher(TaskContext& tc):
    m_registeredNodes(),
    m_parent(tc)
{

}

CanOpenDispatcher::~CanOpenDispatcher()
{
    map< nodeID_t, nodeRegistration_t* >::iterator it;

    for ( it = m_registeredNodes.begin(); it!=m_registeredNodes.end(); ++it)
    {
        nodeID_t nodeId = (*it).first;
        nodeRegistration_t* registration = (*it).second;

        registration->outBootUp.disconnect();
        registration->outNmtState.disconnect();
        registration->inRequestNmt.disconnect();

        m_registeredNodes.erase(nodeId);

        free(registration);
    }
}

bool CanOpenDispatcher::ooRegisterNewNode(CanNodeIdCard node)
{
    pair<map< nodeID_t,nodeRegistration_t* >::iterator,bool> insertReturn;
    nodeRegistration_t* nodeRegistration = new nodeRegistration_t();
    nodeRegistration->task = node.task;


    LOG(Info) << "ooRegisterNewNode node : "
            << "(id=" << node.nodeId << ",task=" << node.task << ",inNmtState" << node.inNmtState << ",inBootUpFrame=" << node.inBootUpFrame << ")" << endlog();

    if( node.check() == false )
    {
        LOG(Error) << "ooRegisterNewNode : inputs are not correct nodeIdCard :" << node.nodeId << endlog();
        goto failedCheck;
    }

    //if inputs are good, do the registration job
    //register a nodeRegistration from the CanNodeIdCard
    insertReturn = m_registeredNodes.insert(
            pair< nodeID_t,nodeRegistration_t* >(
                    node.nodeId, nodeRegistration
            ));
    if( insertReturn.second == false )
    {
        LOG(Error) << "ooRegisterNewNode failed to register : could not insert nodeRegistration in the map for node 0x" << std::hex << node.nodeId << endlog();
        goto failedInsert;
    }

    //connect the NMT state port
    if( !node.inNmtState->connectTo(&(nodeRegistration->outNmtState),
            ConnPolicy::buffer(10, ConnPolicy::LOCK_FREE, true, false)) )
    {
        LOG(Error) << "ooRegisterNewNode failed to register : inNmtState failed to connect to nodeRegistration->outNmtState node:0x" << std::hex << node.nodeId << endlog();
        goto failedInsert;
    }

    //connect the Boot up port
    if( !node.inBootUpFrame->connectTo(&(nodeRegistration->outBootUp)) )
    {
        LOG(Error) << "ooRegisterNewNode failed to register : inBootUpFrame failed to connect to nodeRegistration->outBootUp node:0x" << std::hex << node.nodeId << endlog();
        goto failedInsert;
    }

    //connect the NMT request port
    if( !node.outRequestNmtState->connectTo(&(nodeRegistration->inRequestNmt)) )
    {
        LOG(Error) << "ooRegisterNewNode failed to register : outRequestNmtState failed to connect to nodeRegistration->inRequestNmt node:0x" << std::hex << node.nodeId << endlog();
        goto failedInsert;
    }

    //if we reached this part of the code, the result is correct
    LOG(Info) << "ooRegisterNewNode registered node 0x" << std::hex << node.nodeId << endlog();
    goto success;


    success:
        return true;
    failedInsert:
        m_registeredNodes.erase(node.nodeId);
    failedCheck:
        free(nodeRegistration);
        return false;
}

bool CanOpenDispatcher::ooUnregisterNode(nodeID_t nodeId)
{
    map< nodeID_t, nodeRegistration_t* >::iterator itPort;

    LOG(Info) << "Unregistering node : 0x" << std::hex << nodeId << endlog();

    //check inputs
    if( nodeId < 0 || nodeId > 128 )
    {
        LOG(Error) << "ooRegisterNewNode has received a wrong node number 0x" << std::hex << nodeId << endlog();
        goto failed;
    }
    else if( nodeId == 0x00 || nodeId == 0x01 || nodeId == 0xFF )
    {
        LOG(Warning) << "ooRegisterNewNode has received a reserved node 0x" << std::hex << nodeId << endlog();
    }

    //on recherche un noeud enregistré sous le nodeID reçu
     itPort = m_registeredNodes.find(nodeId);
     if( itPort == m_registeredNodes.end() )
     {
         LOG(Info) << "ooUnregisterNewNode : attempt to unregister a not existing node ! 0x" << std::hex << nodeId << " has sent a boot up frame but no one is listening :(" << endlog();
         goto success;
     }

     if( m_registeredNodes.erase(nodeId) != 1 )
     {
         LOG(Fatal) << "ooUnregisterNewNode : fail to erase the node ! " << std::hex << nodeId << endlog();
         goto failed;
     }
     else
     {
         goto success;
     }

     success:
        return true;
    failed:
        return false;
}

void CanOpenDispatcher::dispatchBootUp(nodeID_t propNodeId, InputPort<nodeID_t>& inBootUpReceived)
{
    nodeID_t nodeIDOfBootedDevice;
    map< nodeID_t, nodeRegistration_t* >::iterator itPort;

    //Traitements des bootUp reçus dans le port connecté à la callback CanFestival
    while( inBootUpReceived.read(nodeIDOfBootedDevice) == NewData )
    {
        //on ne dispatch pas le boot up du master :)
        if( nodeIDOfBootedDevice != propNodeId )
        {
            //on recherche un noeud enregistré sous le nodeID reçu
            itPort = m_registeredNodes.find(nodeIDOfBootedDevice);
            if( itPort == m_registeredNodes.end() )
            {
                LOG(Info) << "dispatchBootUpFrame : nodeId 0x" << std::hex << nodeIDOfBootedDevice << " has sent a boot up frame but no one is listening :(" << endlog();
            }
            else
            {
                LOG(Debug) << "dispatchBootUpFrame : bootUp from nodeId 0x" << std::hex << nodeIDOfBootedDevice << endlog();
                (*itPort).second->outBootUp.write(true);
                ((*itPort).second)->outNmtState.write(Pre_operational);
            }
        }
    }
}

void CanOpenDispatcher::dispatchNmtState()
{
    map< nodeID_t, nodeRegistration_t* >::iterator itPort;
    e_nodeState nodeState = Unknown_state;
    nodeID_t foundNodeId;
    enum_DS301_nmtStateRequest nmtStateCmd;

    //Traitement des états NMT
    //pour tous les nodes enregistrés
    for ( itPort = m_registeredNodes.begin(); itPort!=m_registeredNodes.end(); ++itPort)
    {
        foundNodeId = (*itPort).first;


        if( foundNodeId < 0 || foundNodeId >= 0xFF )
        {
            LOG(Error) << "dispatchNmtState : Node 0x" <<  std::hex << foundNodeId  << " is out of NMT table bounds " << endlog();
        }
        else
        {
        	if( (*itPort).second->inRequestNmt.readNewest(nmtStateCmd) == NewData )
        	{
			   //vérification du paramètre de requete NMT
				if( nmtStateCmd == UnknownRequest )
				{
					LOG(Error) << "coMasterSetNmtNodeState failed you can not ask for the UnknownRequest" << endlog();
				}
				else
				{
					//envoit de la demande d'état
					EnterMutex();
					int cmdResult = masterSendNMTstateChange(&CanARD_Data, (UNS8) foundNodeId, (UNS8) nmtStateCmd);
					LeaveMutex();
					if( cmdResult )
					{
						LOG(Error) << "dispatchNmtState : failed send NMT request 0x" << std::hex << foundNodeId << ";" << nmtStateCmd << ")" << endlog();
					}
				}
        	}


            //on récupère leur état NMT dans la table
        	EnterMutex();
        	nodeState = CanARD_Data.NMTable[foundNodeId];
        	masterRequestNodeState (&CanARD_Data, (UNS8) foundNodeId);
            LeaveMutex();
            (*itPort).second->outNmtState.write(nodeState);
        }
    }
}

void CanOpenDispatcher::unRegisterAll()
{
	 LOG(Info) << "Unregistering all nodes" << endlog();

	map< nodeID_t, nodeRegistration_t* >::iterator it;
	for ( it = m_registeredNodes.begin(); it!=m_registeredNodes.end(); ++it)
	{
		ooUnregisterNode((*it).first);
	}
}

void CanOpenDispatcher::ooPrintRegisteredNodes()
{
    map< nodeID_t, nodeRegistration_t* >::iterator it;

    cout << "------------------------- " << endl;
    cout << "Registered Nodes : " << endl;
    for ( it = m_registeredNodes.begin(); it!=m_registeredNodes.end(); ++it)
    {
        cout << "NodeId 0x=" << std::hex << (*it).first << " named " << (*it).second->task->getName() << " with task=" << std::hex << (*it).second->task << endl;
    }
    cout << "------------------------- " << endl;
}


