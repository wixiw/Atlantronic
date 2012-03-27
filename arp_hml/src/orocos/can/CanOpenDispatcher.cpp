/*
 * CanOpenDispatcher.cpp
 *
 *  Created on: 28 mars 2011
 *      Author: wla
 */

#include "CanOpenDispatcher.hpp"
#include "orocos/can/dictionnary/CanARD.h"
#include <rtt/extras/SlaveActivity.hpp>

using namespace arp_hml;
using namespace RTT;
using namespace extras;

CanOpenDispatcher::CanOpenDispatcher(TaskContext& tc):
    m_registeredNodes(),
    m_parent(tc)
{
	setColog(tc.getOperation("coLog"));
}

void CanOpenDispatcher::setColog(OperationCaller<bool(LoggerLevel,string)> colog)
{
	m_coLog = colog;
}

CanOpenDispatcher::~CanOpenDispatcher()
{
    map< nodeID_t, nodeRegistration_t* >::iterator it;

    for ( it = m_registeredNodes.begin(); it!=m_registeredNodes.end(); ++it)
    {
        nodeID_t nodeId = (*it).first;
        nodeRegistration_t* registration = (*it).second;

        registration->outBootUp.disconnect();

        m_registeredNodes.erase(nodeId);

        free(registration);
    }
}

bool CanOpenDispatcher::ooRegisterNewNode(CanNodeIdCard node)
{
    pair<map< nodeID_t,nodeRegistration_t* >::iterator,bool> insertReturn;
    nodeRegistration_t* nodeRegistration = new nodeRegistration_t();
    nodeRegistration->task = node.task;
    stringstream s;

    if( node.check() == false )
    {
    	s.str("");
    	s << "ooRegisterNewNode : inputs are not correct nodeIdCard :" + node.nodeId;
    	m_coLog(Error,s.str());
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
    	s.str("");
		s  << "ooRegisterNewNode failed to register : could not insert nodeRegistration in the map for node 0x" << std::hex << node.nodeId;
    	m_coLog(Error,s.str());
        goto failedInsert;
    }

    //connect the Boot up port
    if( !node.inBootUpFrame->connectTo(&(nodeRegistration->outBootUp)) )
    {
    	s.str("");
		s  <<  "ooRegisterNewNode failed to register : inBootUpFrame failed to connect to nodeRegistration->outBootUp node:0x" << std::hex << node.nodeId << endlog();
        m_coLog(Error,s.str());
        goto failedInsert;
    }

    //if we reached this part of the code, the result is correct
    s.str("");
	s  <<  "ooRegisterNewNode has registered a new node : "
            << "(id=0x" << std::hex << node.nodeId << ",task=" << node.task->getName() << ")" << endlog();
    m_coLog(Info,s.str());
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
    stringstream s;
    s.str("");
	s  <<  "Unregistering node : 0x" << std::hex << nodeId ;
    m_coLog(Info,s.str());

    //check inputs
    if( nodeId < 0 || nodeId > 128 )
    {
    	s.str("");
		s  << "ooRegisterNewNode has received a wrong node number 0x" << std::hex << nodeId << endlog();
        m_coLog(Error,s.str());
        goto failed;
    }
    else if( nodeId == 0x00 || nodeId == 0x01 || nodeId == 0xFF )
    {
    	s.str("");
		s  << "ooRegisterNewNode has received a reserved node 0x" << std::hex << nodeId << endlog();
        m_coLog(Warning,s.str());
    }

    //on recherche un noeud enregistré sous le nodeID reçu
     itPort = m_registeredNodes.find(nodeId);
     if( itPort == m_registeredNodes.end() )
     {
    	 s.str("");
    	 s << "ooUnregisterNewNode : attempt to unregister a not existing node ! 0x" << std::hex << nodeId << " has sent a boot up frame but no one is listening :(" << endlog();
         m_coLog(Info,s.str());
         goto success;
     }

     if( m_registeredNodes.erase(nodeId) != 1 )
     {
    	 s.str("");
    	 s << "ooUnregisterNewNode : fail to erase the node ! " << std::hex << nodeId << endlog();
         m_coLog(Fatal,s.str());
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
    stringstream s;

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
            	s.str("");
				s  << "dispatchBootUpFrame : nodeId 0x" << std::hex << nodeIDOfBootedDevice << " has sent a boot up frame but no one is listening :(" << endlog();
                m_coLog(Info,s.str());
            }
            else
            {
            	s.str("");
				s  << "dispatchBootUpFrame : bootUp from nodeId 0x" << std::hex << nodeIDOfBootedDevice << endlog();
                m_coLog(Debug,s.str());
                (*itPort).second->outBootUp.write(true);
            }
        }
    }
}

void CanOpenDispatcher::unRegisterAll()
{
	 stringstream s;
	 s << "Unregistering all nodes" << endlog();
	 m_coLog(Info,s.str());

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


