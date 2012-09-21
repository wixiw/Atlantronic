/*
 * Monitor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 */
#include <ros/package.h>
#include <rtt/Component.hpp>
#include <rtt/base/InputPortInterface.hpp>
#include <pthread.h>
#include "Monitor.hpp"
#include "arp_core_version.h"

using namespace arp_core;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_core::Monitor )

Monitor::Monitor(const std::string& name) :
    ARDTaskContext(name, ros::package::getPath("arp_core")),
    propParallelStart(false)
{
    addProperty("propParallelStart",propParallelStart)
            .doc("Set this to true a parallelize the configure/start/stop/cleanup calls on monitored peers");

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
    vector< SendHandle<bool(void)> > operationsSentHandles;
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
            OperationCaller<bool(void)> opConfig = tc->getOperation("configure");
            LOG(Info) << "Configuring " << tc->getName() << endlog();

            if( propParallelStart == true )
            {
                //"send" the operation, this is a non-blocking call on configure(). so the result is not available here and will be poll later.
                operationsSentHandles.push_back(opConfig.send());
            }
            else
            {
                res &= opConfig();
            }
        }
    }

    //only in case of parrallel work, we have to poll and check results
    if( propParallelStart == true )
    {
        LOG(Info) << "Parallised  check of result." << endlog();
        res &= checkSendHandle(operationsSentHandles);
    }

    if( res )
        LOG(Info) << "All peers configured successfully" << endlog();

    return res;
}

bool Monitor::startHook()
{
    bool res =  ARDTaskContext::startHook();
    vector< SendHandle<bool(void)> > operationsSentHandles;
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
            OperationCaller<bool(void)> opConfig = tc->getOperation("start");
            LOG(Info) << "Starting " << tc->getName() << endlog();

            if( propParallelStart == true )
            {
                //"send" the operation, this is a non-blocking call on start(). so the result is not available here and will be poll later.
                operationsSentHandles.push_back(opConfig.send());
            }
            else
            {
                res &= opConfig();
            }
        }
    }

    //only in case of parrallel work, we have to poll and check results
    if( propParallelStart == true )
    {
        LOG(Info) << "Parallised  check of result." << endlog();
        res &= checkSendHandle(operationsSentHandles);
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
                LOG(Debug) << tc->getName() << " is still not Running ! (error)" << endlog();
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
    vector< SendHandle<void(void)> > operationsSentHandles;
    vector<TaskContext*>::iterator i;

    for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredList should not contain null values ! (stop)" << endlog();
        }
        else
        {
            OperationCaller<void(void)> opConfig = tc->getOperation("stop");
            LOG(Info) << "Configuring " << tc->getName() << endlog();

            if( propParallelStart == true )
            {
                //"send" the operation, this is a non-blocking call on stop(). so the result is not available here and will be poll later.
                operationsSentHandles.push_back(opConfig.send());
            }
            else
            {
                opConfig();
            }
        }
    }
}

void Monitor::cleanupHook()
{
    ARDTaskContext::cleanupHook();
   vector< SendHandle<void(void)> > operationsSentHandles;
   vector<TaskContext*>::iterator i;

   for ( i = m_monitoredList.begin() ; i != m_monitoredList.end() ; i++ )
   {
       TaskContext* tc = (*i);

       if( tc == NULL )
       {
           LOG(Error) << "m_monitoredList should not contain null values ! (cleanup)" << endlog();
       }
       else
       {
           OperationCaller<void(void)> opConfig = tc->getOperation("cleanup");
           LOG(Info) << "Configuring " << tc->getName() << endlog();

           if( propParallelStart == true )
           {
               //"send" the operation, this is a non-blocking call on cleanup(). so the result is not available here and will be poll later.
               operationsSentHandles.push_back(opConfig.send());
           }
           else
           {
               opConfig();
           }
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

bool Monitor::checkSendHandle(vector< SendHandle<bool(void)> > operationsSentHandles)
{
    bool res = true;

    bool oneOperationIsNotReady = true;
    vector< SendHandle<bool(void)> >::iterator h;

    //check operation sent handles to check if all operations are finished.
    while(oneOperationIsNotReady)
    {
        oneOperationIsNotReady = false;

        //we wait 1s to let time to peers to process the sent operation.
        sleep(1);

        for ( h = operationsSentHandles.begin() ; h != operationsSentHandles.end() ; h++ )
        {
            SendHandle<bool(void)> handle = *h;

            //if one operation has not finished to be processed, we continue to wait.
            if( handle.collect() == SendNotReady )
            {
                oneOperationIsNotReady = true;
                break;
            }
        }
    }

    //now operation are processed, we check if they succeed
    for ( h = operationsSentHandles.begin() ; h != operationsSentHandles.end() ; h++ )
    {
        bool operationReturn = false;
        SendHandle<bool(void)> handle = *h;

        //check if they where correctly processed
        if( handle.collect(operationReturn) != SendSuccess )
        {
            LOG(Error) << "Monitor::configure failed to call one peer configure() operation for unknown reason."<< endlog();
            res = false;
        }

        //check the return value
        if( operationReturn == false )
            res = false;
    }

    return res;
}
