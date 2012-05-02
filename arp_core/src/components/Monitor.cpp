/*
 * Monitor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 */
#include <ros/package.h>
#include <rtt/Component.hpp>
#include <rtt/base/InputPortInterface.hpp>

#include "Monitor.hpp"
#include "arp_core_version.h"

using namespace arp_core;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_core::Monitor )

Monitor::Monitor(const std::string& name) :
    ARDTaskContext(name, ros::package::getPath("arp_core"))
{
    addOperation("ooAddMonitoredPeer", &Monitor::addMonitoredPeer, this, OwnThread)
            .doc("Add a peer to the monitored list")
            .arg("peerName","Name of the peer to add to the list");

    addOperation("coDisplayMonitoredPeers", &Monitor::displayMonitoredPeers, this, ClientThread)
               .doc("Display the list of peers");
    addOperation("coGetCoreVersion",&Monitor::coGetCoreVersion, this, ClientThread)
            .doc("Returns a string containing Core version");
    addOperation("connect",&Monitor::connect, this, ClientThread)
            .doc("Use this to connect internal peers");
}

Monitor::~Monitor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool Monitor::configureHook()
{
    bool res = ARDTaskContext::configureHook();

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (configure)" << endlog();
            res = false;
        }
        else
        {
            LOG(Info) << "Configuring " << tc->getName() << endlog();
            res &= tc->configure();
        }
    }

    if( res )
        LOG(Info) << "All peers configured successfully" << endlog();

    return res;
}

bool Monitor::startHook()
{
    bool res = ARDTaskContext::startHook();

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (start)" << endlog();
            res = false;
        }
        else
        {
            LOG(Info) << "Starting " << tc->getName() << endlog();
            res &= tc->start();
        }
    }

    if( res )
        LOG(Info) << "All peers started successfully" << endlog();

    return res;
}

void Monitor::updateHook()
{
    ARDTaskContext::updateHook();
    bool someoneIsNotRunning = false;

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (update)" << endlog();
            error();
        }
        else
        {
            if( !tc->isRunning() )
            {
                LOG(Error) << tc->getName() << " is no more Running ! (update)" << endlog();
                someoneIsNotRunning |= true;
            }
        }
    }

    //in order to avoid to miss stopped component after the first one, we let the loop going on and logging faulty components name
    //anyway if at least one is not running we go in error state.
    if( someoneIsNotRunning )
    {
        error();
    }
}

void Monitor::errorHook()
{
    ARDTaskContext::errorHook();
    bool isEveryOneRunning = true;

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (error)" << endlog();
            error();
        }
        else
        {
            if( !tc->isRunning() )
            {
                LOG(Error) << tc->getName() << " is still not Running ! (error)" << endlog();
                isEveryOneRunning = false;
            }
        }
    }

    if( isEveryOneRunning )
    {
        recover();
    }
}

void Monitor::stopHook()
{
    ARDTaskContext::stopHook();

    vector<TaskContext*>::reverse_iterator  i;
    for ( i = m_monitoredList.rbegin() ; i != m_monitoredList.rend() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (stop)" << endlog();
        }
        else
        {
            LOG(Info) << "Stopping " << tc->getName() << endlog();
            tc->stop();
        }
    }
}

void Monitor::cleanupHook()
{
   ARDTaskContext::cleanupHook();

   vector<TaskContext*>::reverse_iterator  i;
   for ( i = m_monitoredList.rbegin() ; i != m_monitoredList.rend() ; i++ )
   {
       TaskContext* tc = (*i);

       if( tc == NULL )
       {
           LOG(Error) << "m_monitoredList should not contain null values ! (cleanup)" << endlog();
       }
       else
       {
           LOG(Info) << "Cleaning " << tc->getName() << endlog();
           tc->cleanup();
       }
   }
}


//-----------------------------------------------------

bool Monitor::addMonitoredPeer(std::string peerName )
{
    bool res = true;

    if( ! hasPeer(peerName) )
    {
        LOG(Error) << "You can't monitor a component that is not your peer !" << endlog();
        res = false;
    }
    else if( getTaskState() !=  getPeer(peerName)->getTaskState() )
    {
        LOG(Error) << "You can't add a new monitored component that is not in your current state !" << endlog();
        res = false;
    }
    else
    {
        vector<TaskContext*>::iterator i;
        for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
        {
            TaskContext* tc = (*i);
            if( tc == NULL )
            {
              LOG(Error) << "m_monitoredList should not contain null values ! (addMonitoredPeer)" << endlog();
              res = false;
            }
            else
            {
              if( tc->getName() == peerName )
              {
                  LOG(Error) << tc->getName() << " is already in the list !" << endlog();
                  res = false;
                  break;
              }
            }
        }

        if( res == true )
        {
            LOG(Info) << "New peer to monitor : " << peerName << endlog();
            m_monitoredList.push_back (getPeer(peerName));
        }
    }

    return res;
}


void Monitor::displayMonitoredPeers()
{
    cout << endl;
    cout << "List of monitored peers : " << endl;
    cout << endl;

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_monitoredList should not contain null values ! (displayMonitoredPeers)" << endl;
        }
        else
        {
            cout << tc->getName() << endl;
        }
    }

    cout << "------------------------" << endl;
    cout << endl;
}

bool Monitor::checkPortConnection()
{
    bool res = true;

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_monitoredList should not contain null values ! (checkPortConnection)" << endl;
        }
        else
        {
            RTT::DataFlowInterface::Ports ports = tc->ports()->getPorts();
            RTT::DataFlowInterface::Ports::iterator p;
            for( p = ports.begin() ; p != ports.end() ; p++ )
            {
                base::PortInterface* port = (*p);
                if( dynamic_cast< RTT::base::InputPortInterface*>(port) != 0 && port->connected() == false )
                {
                    LOG(Error)  << "checkPortConnection : " << tc->getName() << "." << port->getName() << " is not connected !" << endlog();
                    res &= false;
                }
            }
        }
    }

    return res;
}

string Monitor::coGetCoreVersion()
{
    return ARP_CORE_VERSION;
}

bool Monitor::connect(const std::string& compA, const std::string& portA, const std::string& compB, const std::string& portB)
{
    TaskContext* tcA;
    RTT::base::PortInterface* portItfA;
    TaskContext* tcB;
    RTT::base::PortInterface* portItfB;

    tcA = this->getPeer(compA);
    if( tcA == NULL )
    {
        LOG(Error)  << "connect : Failed to find component : " << compA << endlog();
        goto failed;
    }
    portItfA = tcA->getPort(portA);
    if( portItfA == NULL )
    {
        LOG(Error)  << "connect : Failed to find port : " << portA << " in component " << compA << endlog();
        goto failed;
    }

    tcB = this->getPeer(compB);
    if( tcB == NULL )
    {
        LOG(Error)  << "connect : Failed to find component : " << compB << endlog();
        goto failed;
    }
    portItfB = tcB->getPort(portB);
    if( portItfB == NULL )
    {
        LOG(Error)  << "connect : Failed to find port : " << portB << " in component " << compB << endlog();
        goto failed;
    }

    if( portItfA->connectTo(portItfB) == false )
    {
        LOG(Error)  << "connect : Failed to connect ports : " << compA << "." << portA << " to " << compB << "." << portB << endlog();
        goto failed;
    }

    LOG(Info)  << "Connecting " << compA << "." << portA << " to " << compB << "." << portB << endlog();

    goto success;

    failed:
        return false;
    success:
        return true;
}
