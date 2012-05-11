/*
 * MotionScheduler.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: ard
 */

#include <rtt/Component.hpp>
#include <rtt/typekit/Types.hpp>
#include <rtt/marsh/Marshalling.hpp>
#include <ros/package.h>
#include <math/core>
#include <fstream>

#include "MotionScheduler.hpp"

using namespace arp_core;
using namespace arp_math;
using namespace RTT;
using namespace base;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_core::MotionScheduler )

MotionScheduler::MotionScheduler(const std::string& name):
        FBSched(name),
        propTimeReporting(false),
        m_timer(3000)
{
    addProperty("propTimeReporting",propTimeReporting);
    addPort("inClock",inClock);
    addOperation("timeReport", &MotionScheduler::timeReport, this, ClientThread).doc("Computes and prints time reports.");
}

bool MotionScheduler::configureHook()
{
    bool res = FBSched::configureHook();

    boost::shared_ptr<RTT::Marshalling> marshalling = this->getProvider<RTT::Marshalling>("marshalling");

    if( marshalling != NULL)
    {
        string fileName = ros::package::getPath("arp_hml") + "/script/orocos/conf/" + getName() + ".xml";

        if( std::ifstream( fileName.c_str() ) )
        {
            if( marshalling->updateProperties(fileName) == false )
            {
                log(Error) << "Configure failed, error while reading " << fileName << endlog();
                res &= false;
            }
            //on a reussit à charger les propriétés !
            else
            {
                log(Info) << "Properties loaded successfully from " << fileName << endlog();
                res &= true;
            }
        }
    }
    else
    {
        log(Error) << "Failed to load marshalling service" << endlog();
        res = false;
    }

    return res;
}

bool MotionScheduler::startHook()
{
    bool res = FBSched::startHook();

    if( propTimeReporting )
    {
        m_timer.Start();
        m_timer.Stop();
        m_timer.ResetStat();
    }

    return res;
}

void MotionScheduler::updateHook()
{
    if( propTimeReporting )
        m_timer.Start();

    FBSched::updateHook();

    if( propTimeReporting )
        m_timer.Stop();
}

void MotionScheduler::stopHook()
{

}

void MotionScheduler::timeReport()
{
    if( !isRunning() || !propTimeReporting )
        cout << "Time Stats are disabled. The component must be in running state with propTimereporting=true." << endl;
    else
        cout << m_timer.GetReport() << endl;
}

MotionScheduler::~MotionScheduler()
{
}
