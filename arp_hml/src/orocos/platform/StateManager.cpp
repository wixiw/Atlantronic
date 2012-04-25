/*
 * StateManager.cpp
 *
 *  Created on: 12 Fev. 2012
 *      Author: ard, wla
 */
#include "StateManager.hpp"

using namespace arp_hml;
using namespace arp_core;
using namespace std;
using namespace RTT;

StateManager::StateManager(ARDTaskContext& owner):
    m_owner(owner)
{
    m_owner.addPort("inLeftSteeringHomingDone",inLeftSteeringHomingDone)
            .doc("");
    m_owner.addPort("inRightSteeringHomingDone",inRightSteeringHomingDone)
            .doc("");
    m_owner.addPort("inRearSteeringHomingDone",inRearSteeringHomingDone)
            .doc("");

    m_owner.addPort("outHomingDone",outHomingDone)
            .doc("");

    m_owner.addOperation("ooSetDrivingOperationMode", &StateManager::ooSetDrivingOperationMode, this, OwnThread)
            .doc("Choose a mode of operation on driving motors")
            .arg("state","choose between spped,position,torque,other");
    m_owner.addOperation("ooSetSteeringOperationMode", &StateManager::ooSetSteeringOperationMode, this, OwnThread)
            .doc("Choose a mode of operation on sttering motors")
            .arg("state","choose between spped,position,torque,other");
}

bool StateManager::configureHook()
{
    bool res = true;

    base::PropertyBase* property;

    property= m_owner.getProperty("propRequireCompleteHardware");
    m_propRequireCompleteHardware = dynamic_cast< RTT::Property<bool>*>(property);
    if( property == NULL || m_propRequireCompleteHardware == NULL )
    {
        LOGS(Error) << "Failed to get propRequireCompleteHardware property" << endlog();
        res = false;
    }
    else
    {
        res &= getPeersOperations();
    }

    return res;
}

void StateManager::updateHook()
{
    bool leftDone,rightDone,rearDone;
    inLeftSteeringHomingDone.readNewest(leftDone);
    inRightSteeringHomingDone.readNewest(rightDone);
    inRearSteeringHomingDone.readNewest(rearDone);

    outHomingDone.write(leftDone && rightDone && rearDone);
}

bool StateManager::getPeersOperations()
{
    bool res = true;

    //we test the peer existence, because in some cases we don't care in case of incomplete hardware
    if( m_owner.hasPeer("LeftDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("LeftDriving" ,     "ooSetOperationMode",   m_ooSetLeftDrivingOperationMode);
    }
    if( m_owner.hasPeer("RightDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RightDriving",     "ooSetOperationMode",   m_ooSetRightDrivingOperationMode);
    }
    if( m_owner.hasPeer("RearDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RearDriving",      "ooSetOperationMode",   m_ooSetRearDrivingOperationMode);
    }
    if( m_owner.hasPeer("LeftSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("LeftSteering" ,    "ooSetOperationMode",   m_ooSetLeftSteeringOperationMode);
    }
    if( m_owner.hasPeer("RightSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RightSteering",    "ooSetOperationMode",   m_ooSetRightSteeringOperationMode);
    }
    if( m_owner.hasPeer("RearSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RearSteering",     "ooSetOperationMode",   m_ooSetRearSteeringOperationMode);
    }

    return res;
}

//--------------------------------------------------------------------------------------------------------

bool StateManager::ooSetDrivingOperationMode(const string state)
{
    //remise en mode vitesse des moteurs
    if( m_ooSetLeftDrivingOperationMode(state) == false
     || m_ooSetRightDrivingOperationMode(state) == false
     || m_ooSetRearDrivingOperationMode(state) == false
     )
    {
        LOGS(Error) << "ooSetDrivingOperationMode : could not switch back to " << state << " speed mode." << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetDrivingOperationMode : Motors mode of operation is " << state << endlog();
    goto success;


    failed:
        return false;
    success:
        return true;
}

bool StateManager::ooSetSteeringOperationMode(const string state)
{
    //remise ne mode position des moteurs
    if( m_ooSetLeftSteeringOperationMode(state) == false
    || m_ooSetRightSteeringOperationMode(state) == false
    || m_ooSetRearSteeringOperationMode(state) == false
    )
    {
        LOGS(Error) << "ooSetSteeringOperationMode : could not switch back to " << state << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetSteeringOperationMode : Motors mode of operation is " << state << " properly." << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}

