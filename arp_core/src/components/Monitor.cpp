/*
 * Monitor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 */
#include <ros/package.h>
#include <rtt/Component.hpp>
#include "Monitor.hpp"

using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_core::Monitor )

Monitor::Monitor(const std::string& name) :
    ARDTaskContext(name, ros::package::getPath("arp_core"))
{
    addOperation("ooAddMonitoredPeer", &Monitor::addMonitoredPeer, this, OwnThread)
            .doc("Add a peer to the monitored list")
            .arg("peerName","Name of the peer to add to the list");

    addOperation("coDisplayMonitoredPeers", &Monitor::displayMonitoredPeers, this, ClientThread)
               .doc("Display the list of peers");


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
            res &= tc->configure();
        }
    }

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
            res &= tc->start();
        }
    }

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
