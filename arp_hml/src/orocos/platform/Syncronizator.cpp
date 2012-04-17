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
using namespace Eigen;

ORO_LIST_COMPONENT_TYPE( arp_hml::Syncronizator )

Syncronizator::Syncronizator(const std::string name):
    HmlTaskContext(name),
    attrNbError(0.0),
    propVerbose(false)
{
    createOrocosInterface();
}

void Syncronizator::eventCanSyncCB(RTT::base::PortInterface* portInterface)
{
     inCanSync.readNewest(m_syncTime);
     m_syncCount = 0;
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
    timespec syncValue;
    InputPort<timespec>* inPort = dynamic_cast< InputPort<timespec>* > (portInterface);
    if( inPort == NULL )
    {
        LOG(Error) << "failed to cast port " << name << " to an InputPort<double>" << endlog();
        return;
    }

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

    inPort->readNewest(syncValue);

    //on ne met à jour le compteur que si la syncronisation reçue est bien la bonne.
    if( syncValue.tv_sec == m_syncTime.tv_sec && syncValue.tv_nsec == m_syncTime.tv_nsec)
    {
        //pour creer le masque binaire correspondant au moteur dont on a reçu le sync il faut
        //ecrire un1 à la ième colonne du mot binaire, soit 1<<i;
        //pour additionner le compteur avec le nouveau masque on fait un ou bit à bit de sorte qu'il
        //y aura à 1 à la ième colonne du mot binaire m_syncCount et les autres colonnes sont inchangées.
        m_syncCount |= 1<<i;
        if( propVerbose )
            LOG(Info) << "portCB " << name << " counter is " << std::hex << m_syncCount << std::dec << endlog();
    }
    else
    {
        //si on est la c'est qu'il se passe des choses etonnantes au niveau du scheduling
        attrNbError++;
    }
}

void Syncronizator::updateHook()
{
    if( m_syncCount == 0b111111 )
    {
        outMotorMeasures.write(readInputs());
        outClock.write(m_syncTime);
        m_syncCount = 0;

        if( propVerbose )
            LOG(Info) << "updateHook" << endlog();
    }
}

MotorState Syncronizator::readInputs()
{
    MotorState motorState;
    inLeftDrivingPosition.readNewest(motorState.driving.left.position);
    inRightDrivingPosition.readNewest(motorState.driving.right.position);
    inRearDrivingPosition.readNewest(motorState.driving.rear.position);
    inLeftSteeringPosition.readNewest(motorState.steering.left.position);
    inRightSteeringPosition.readNewest(motorState.steering.right.position);
    inRearSteeringPosition.readNewest(motorState.steering.rear.position);

    inLeftDrivingVelocity.readNewest(motorState.driving.left.velocity);
    inRightDrivingVelocity.readNewest(motorState.driving.right.velocity);
    inRearDrivingVelocity.readNewest(motorState.driving.rear.velocity);
    inLeftSteeringVelocity.readNewest(motorState.steering.left.velocity);
    inRightSteeringVelocity.readNewest(motorState.steering.right.velocity);
    inRearSteeringVelocity.readNewest(motorState.steering.rear.velocity);

    inLeftDrivingTorque.readNewest(motorState.driving.left.torque);
    inRightDrivingTorque.readNewest(motorState.driving.right.torque);
    inRearDrivingTorque.readNewest(motorState.driving.rear.torque);
    inLeftSteeringTorque.readNewest(motorState.steering.left.torque);
    inRightSteeringTorque.readNewest(motorState.steering.right.torque);
    inRearSteeringTorque.readNewest(motorState.steering.rear.torque);

    return motorState;
}

void Syncronizator::createOrocosInterface()
{
    addAttribute("attrNbError",attrNbError);

    addProperty("propVerbose",propVerbose)
            .doc("Use this to have detailed report about synchronisation");

    addPort("outClock",outClock)
                .doc("");
    addPort("outMotorMeasures",outMotorMeasures)
                .doc("");
    addEventPort("inCanSync",inCanSync, boost::bind(&Syncronizator::eventCanSyncCB,this,_1))
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
