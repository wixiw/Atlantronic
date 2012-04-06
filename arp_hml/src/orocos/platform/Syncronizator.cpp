/*
 * Syncronizator.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#include "Syncronizator.hpp"
#include <rtt/Component.hpp>

using namespace arp_core;
using namespace arp_model;
using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::Syncronizator )

#define RESET_SYNC() \
		m_syncCount = 0; \
	    m_syncTimes[0] = -1;\
	    m_syncTimes[1] = -1;\
	    m_syncTimes[2] = -1;\
	    m_syncTimes[3] = -1;\
	    m_syncTimes[4] = -1;\
	    m_syncTimes[5] = -1;

Syncronizator::Syncronizator(const std::string name):
    HmlTaskContext(name)
{
    RESET_SYNC();
    createOrocosInterface();
}

void Syncronizator::eventPortCB(RTT::base::PortInterface* portInterface)
{
    if( portInterface == NULL )
    {
        LOG(Fatal) << "eventPortCB Orocos is bullshitting" << endlog();
        return;
    }

    const char* name = portInterface->getName().c_str();
    int i = 0;

    //enregistre la date du port triggé dans le tableau des sync.
    //pour ce faire on cherche a quel index il correspond.
    if( strcmp(name, "inLeftDrivingClock") == 0 )
        i = 0;
    else if( strcmp(name, "inRightDrivingClock") == 0 )
        i = 1;
    else if( strcmp(name, "inRearDrivingClock") == 0 )
        i = 2;
    else if( strcmp(name, "inLeftSteeringClock") == 0 )
        i = 3;
    else if( strcmp(name, "inRightSteeringClock") == 0 )
        i = 4;
    else if( strcmp(name, "inRearSteeringClock") == 0 )
        i = 5;

    //m_eventCbMutex.lock();
    inLeftDrivingClock.readNewest(m_syncTimes[i]);
    m_syncCount++;
    //m_eventCbMutex.unlock();

    //LOG(Info) << "eventPortCB " << portInterface->getName() << "time : " << m_syncTimes[i] << endlog();
}

void Syncronizator::updateHook()
{
    double sync;

    //m_eventCbMutex.lock();
    if( isAllSyncReveived(sync) )
    {
        //LOG(Info) << "updateHook" << endlog();
        outMotorMeasures.write(readInputs());
        outClock.write(sync);
        RESET_SYNC();
    }

    if( m_syncCount > 6 )
    {
        //LOG(Error) << "syncCount should not be greather than 6, this is a scheduling error " << m_syncCount << endlog();
        RESET_SYNC();
    }
    //m_eventCbMutex.unlock();
}

bool Syncronizator::isAllSyncReveived(double& syncTime)
{
    bool res = true;

    if( m_syncCount == 6 )
    {
        for( int i = 0 ; i < 5 ; i++ )
        {
            if( m_syncTimes[i] != m_syncTimes[i+1] )
            {
                res = false;
            }
        }
        if( res == false )
        {
            /*LOG(Fatal) << "syncTimes not consistent ["
                    << m_syncTimes[0] << ","
                    << m_syncTimes[1] << ","
                    << m_syncTimes[2] << ","
                    << m_syncTimes[3] << ","
                    << m_syncTimes[4] << ","
                    << m_syncTimes[5] << "]" <<endlog();*/
            RESET_SYNC();
        }
    }
    else
    {
        res = false;
    }

    if( res == true )
        syncTime = m_syncTimes[0];
    else
        syncTime = -1;

    return res;
}

MotorState Syncronizator::readInputs()
{
    MotorState motorState;
    inLeftDrivingPosition.readNewest(motorState.leftDrivingPosition);
    inRightDrivingPosition.readNewest(motorState.rightDrivingPosition);
    inRearDrivingPosition.readNewest(motorState.rearDrivingPosition);
    inLeftSteeringPosition.readNewest(motorState.leftSteeringPosition);
    inRightSteeringPosition.readNewest(motorState.rightSteeringPosition);
    inRearSteeringPosition.readNewest(motorState.rearSteeringPosition);

    inLeftDrivingVelocity.readNewest(motorState.leftDrivingVelocity);
    inRightDrivingVelocity.readNewest(motorState.rightDrivingVelocity);
    inRearDrivingVelocity.readNewest(motorState.rearDrivingVelocity);
    inLeftSteeringVelocity.readNewest(motorState.leftSteeringVelocity);
    inRightSteeringVelocity.readNewest(motorState.rightSteeringVelocity);
    inRearSteeringVelocity.readNewest(motorState.rearSteeringVelocity);

    inLeftDrivingTorque.readNewest(motorState.leftDrivingTorque);
    inRightDrivingTorque.readNewest(motorState.rightDrivingTorque);
    inRearDrivingTorque.readNewest(motorState.rearDrivingTorque);
    inLeftSteeringTorque.readNewest(motorState.leftSteeringTorque);
    inRightSteeringTorque.readNewest(motorState.rightSteeringTorque);
    inRearSteeringTorque.readNewest(motorState.rearSteeringTorque);

    return motorState;
}

void Syncronizator::createOrocosInterface()
{
    addPort("outClock",outClock)
                .doc("");
    addPort("outMotorMeasures",outMotorMeasures)
                .doc("");

    addEventPort("inLeftDrivingClock",inLeftDrivingClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");
    addEventPort("inRightDrivingClock",inRightDrivingClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");
    addEventPort("inRearDrivingClock",inRearDrivingClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");
    addEventPort("inLeftSteeringClock",inLeftSteeringClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");
    addEventPort("inRightSteeringClock",inRightSteeringClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");
    addEventPort("inRearSteeringClock",inRearSteeringClock, boost::bind(&Syncronizator::eventPortCB,this,_1))
            .doc("");

    addPort("inLeftDrivingVelocity",inLeftDrivingVelocity)
            .doc("");
    addPort("inRightDrivingVelocity",inRightDrivingVelocity)
            .doc("");
    addPort("inRearDrivingVelocity",inRearDrivingVelocity)
            .doc("");
    addPort("inLeftSteeringVelocity",inLeftSteeringVelocity)
            .doc("");
    addPort("inRightSteeringVelocity",inRightSteeringVelocity)
            .doc("");
    addPort("inRearSteeringVelocity",inRearSteeringVelocity)
            .doc("");

    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");
    addPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("");
    addPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("");
    addPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("");
    addPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("");

    addPort("inLeftDrivingTorque",inLeftDrivingTorque)
            .doc("");
    addPort("inRightDrivingTorque",inRightDrivingTorque)
            .doc("");
    addPort("inRearDrivingTorque",inRearDrivingTorque)
            .doc("");
    addPort("inLeftSteeringTorque",inLeftSteeringTorque)
            .doc("");
    addPort("inRightSteeringTorque",inRightSteeringTorque)
            .doc("");
    addPort("inRearSteeringTorque",inRearSteeringTorque)
            .doc("");
}
