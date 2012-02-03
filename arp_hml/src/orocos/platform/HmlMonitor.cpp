/*
 * HmlMonitor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 */
#include <ros/package.h>
#include <rtt/Component.hpp>
#include "HmlMonitor.hpp"
#include "arp_hml_version.h"

using namespace arp_hml;
using namespace arp_core;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::HmlMonitor )

HmlMonitor::HmlMonitor(const std::string& name) :
    Monitor(name),
    m_power(*this)
{
    attrProjectRootPath = ros::package::getPath("arp_hml");

    addOperation("ooAddHmlBusMonitoredPeer", &HmlMonitor::addHmlBusMonitoredPeer, this, OwnThread)
            .doc("Add a peer to the bus monitored list. This list is different because those components are configured and start before the others")
            .arg("peerName","Name of the bus peer to add to the list");
    addOperation("coGetHmlVersion",&HmlMonitor::coGetHmlVersion, this, ClientThread)
            .doc("Returns a string containing HML version");
    addOperation("ooResetHml",&HmlMonitor::ooResetHml, this, OwnThread)
        .doc("Ask all cane node to reset. Could be usefull after an emergency stop");
}

//------------------------------------------------------------------------------------------------------------------

bool HmlMonitor::configureHook()
{
    bool res = true ;

    //configure and start bus monitored components
    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredBusList.begin() ; i != m_monitoredBusList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredBusList should not contain null values ! (configure)" << endlog();
            res = false;
        }
        else
        {
            res &= tc->configure();
            res &= tc->start();
        }
    }

    //configure normal monitored components
    res &= Monitor::configureHook();

    //configure power addon
    m_power.configureHook();

    //get operation to reset hml
    res &= getOperation("WoodheadOut",      "coReset",  m_coResetWoodheadOut);
    res &= getOperation("WoodheadIn",       "coReset",  m_coResetWoodheadIn);
    res &= getOperation("LeftDriving",      "coReset",  m_coResetLeftDriving);
    res &= getOperation("RightDriving",     "coReset",  m_coResetRightDriving);
    res &= getOperation("RearDriving",      "coReset",  m_coResetRearDriving);
    res &= getOperation("LeftSteering",     "coReset",  m_coResetLeftSteering);
    res &= getOperation("RightSteering",    "coReset",  m_coResetRightSteering);
    res &= getOperation("RearSteering",     "coReset",  m_coResetRearSteering);


    return res;
}

void HmlMonitor::updateHook()
{
    Monitor::updateHook();

    bool someoneIsNotRunning = false;
    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredBusList.begin() ; i != m_monitoredBusList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            LOG(Error) << "m_monitoredBusList should not contain null values ! (update)" << endlog();
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

    //manage power on motors
    m_power.updateHook();
}


void HmlMonitor::cleanupHook()
{
    Monitor::cleanupHook();

    //stop and unconfigure busses
   vector<TaskContext*>::reverse_iterator  i;
   for ( i = m_monitoredBusList.rbegin() ; i != m_monitoredBusList.rend() ; i++ )
   {
       TaskContext* tc = (*i);

       if( tc == NULL )
       {
           LOG(Error) << "m_monitoredBusList should not contain null values ! (cleanup)" << endlog();
       }
       else
       {
           tc->stop();
           tc->cleanup();
       }
   }
}


//-----------------------------------------------------

bool HmlMonitor::addHmlBusMonitoredPeer(std::string peerName )
{
    bool res = true;

    if( ! hasPeer(peerName) )
    {
        LOG(Error) << "You can't monitor a component that is not your peer !" << endlog();
        res = false;
    }
    else if( getTaskState() !=  RTT::base::TaskCore::PreOperational
            || getPeer(peerName)->getTaskState() !=  RTT::base::TaskCore::PreOperational )
    {
        LOG(Error) << "You can't add a new bus monitored component if both are not in the unconfigured state !" << endlog();
        res = false;
    }
    else
    {
        vector<TaskContext*>::iterator i;
        for ( i = m_monitoredBusList.begin() ; i != m_monitoredBusList.end() ; i++ )
        {
            TaskContext* tc = (*i);
            if( tc == NULL )
            {
              LOG(Error) << "m_monitoredBusList should not contain null values ! (addHmlMonitoredPeer)" << endlog();
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
            LOG(Info) << "New bus peer to monitor : " << peerName << endlog();
            m_monitoredBusList.push_back (getPeer(peerName));
        }
    }

    return res;
}


void HmlMonitor::displayHmlMonitoredPeers()
{
    cout << endl;
    cout << "List of monitored busses : " << endl;
    cout << endl;

    vector<TaskContext*>::iterator i;
    for ( i = m_monitoredBusList.begin() ; i != m_monitoredBusList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_monitoredBusList should not contain null values ! (displayHmlMonitoredPeers)" << endl;
        }
        else
        {
            cout << tc->getName() << endl;
        }
    }

    HmlMonitor::displayHmlMonitoredPeers();
}

string HmlMonitor::coGetHmlVersion()
{
    return ARP_HML_VERSION;
}

bool HmlMonitor::ooResetHml()
{
    bool res = true;
    res &= m_coResetWoodheadOut();
    res &= m_coResetWoodheadIn();
    res &= m_coResetLeftDriving();
    res &= m_coResetRightDriving();
    res &= m_coResetRearDriving();
    res &= m_coResetLeftSteering();
    res &= m_coResetRightSteering();
    res &= m_coResetRearSteering();
    return false;
}
