/*
 * PowerManager.cpp
 *
 *  Created on: 7 janv. 2012
 *      Author: ard, wla
 */
#include "PowerManager.hpp"

using namespace arp_hml;
using namespace arp_core;

PowerManager::PowerManager(ARDTaskContext& owner):
    propRequireCompleteHardware(false),
    m_owner(owner),
    m_tractionEnableOperationInProgress(false),
    m_directionEnableOperationInProgress(false),
    m_tractionDisableOperationInProgress(false),
    m_directionDisableOperationInProgress(false)
{
    m_owner.addProperty("propRequireCompleteHardware", propRequireCompleteHardware)
        .doc("Decide weather complete hardware must be present or not");

    m_owner.addPort("inLeftDrivingEnable",inLeftDrivingEnable)
            .doc("Left driving soft enable state");
    m_owner.addPort("inRightDrivingEnable",inRightDrivingEnable)
            .doc("Right driving soft enable state");
    m_owner.addPort("inRearDrivingEnable",inRearDrivingEnable)
            .doc("Rear driving soft enable state");
    m_owner.addPort("inLeftSteeringEnable",inLeftSteeringEnable)
            .doc("Left steering soft enable state");
    m_owner.addPort("inRightSteeringEnable",inRightSteeringEnable)
            .doc("Right steering soft enable state");
    m_owner.addPort("inRearSteeringEnable",inRearSteeringEnable)
            .doc("Rear steering soft enable state");

    m_owner.addPort("outDrivingEnable",outDrivingEnable)
            .doc("Driving soft enable state");
    m_owner.addPort("outSteeringEnable",outSteeringEnable)
            .doc("Steering soft enable state");
    m_owner.addPort("outEnable",outEnable)
            .doc("All device are soft enabled");

    m_owner.addOperation("ooEnableTraction", &PowerManager::enableTraction, this, OwnThread)
            .doc("");
    m_owner.addOperation("ooEnableDirection", &PowerManager::enableDirection, this, OwnThread)
            .doc("");
    m_owner.addOperation("ooEnablePower", &PowerManager::enablePower, this, OwnThread)
            .doc("");

    m_owner.addOperation("ooDisableTraction", &PowerManager::disableTraction, this, OwnThread)
            .doc("");
    m_owner.addOperation("ooDisableDirection", &PowerManager::disableDirection, this, OwnThread)
            .doc("");
    m_owner.addOperation("ooDisablePower", &PowerManager::disablePower, this, OwnThread)
            .doc("");
}

//------------------------------------------------------------------------------------------------------------------

bool PowerManager::configure()
{
    bool res = true ;

    //get ppers operations to show a simplier interface to the outside world
    res &= getPeersOperations();

    return res;
}

bool PowerManager::getPeersOperations()
{
    bool res = true;
/*
    //we test the peer existence, because in some cases we don't care in case of incomplete hardware
    if( getOwner()->hasPeer("LeftDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("LeftDriving",      "ooEnableDrive",        m_ooEnableLeftDriving);
    }
    if( getOwner()->hasPeer("RightDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("RightDriving",     "ooEnableDrive",        m_ooEnableRightDriving);
    }
    if( getOwner()->hasPeer("RearDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("RearDriving",      "ooEnableDrive",        m_ooEnableRearDriving);
    }
    if( getOwner()->hasPeer("LeftSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("LeftSteering",     "ooEnableDrive",        m_ooEnableLeftSteering);
    }
    if( getOwner()->hasPeer("RightSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("RightSteering",    "ooEnableDrive",        m_ooEnableRightSteering);
    }
    if( getOwner()->hasPeer("RearSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("RearSteering",     "ooEnableDrive",        m_ooEnableRearSteering);
    }


    if( getOwner()->hasPeer("LeftDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("LeftDriving",      "ooDisableDrive",       m_ooDisableLeftDriving);
    }
    if( getOwner()->hasPeer("RightDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("RightDriving",     "ooDisableDrive",       m_ooDisableRightDriving);
    }
    if( getOwner()->hasPeer("RearDriving") || propRequireCompleteHardware )
    {
        res &= getOperation("RearDriving",      "ooDisableDrive",       m_ooDisableRearDriving);
    }
    if( getOwner()->hasPeer("LeftSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("LeftSteering",     "ooDisableDrive",       m_ooDisableLeftSteering);
    }
    if( getOwner()->hasPeer("RightSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("RightSteering",    "ooDisableDrive",       m_ooDisableRightSteering);
    }
    if( getOwner()->hasPeer("RearSteering") || propRequireCompleteHardware )
    {
        res &= getOperation("RearSteering",     "ooDisableDrive",       m_ooDisableRearSteering);
    }*/

    return res;
}

void PowerManager::update()
{
    bool leftDrivingEnable = false;
    bool rightDrivingEnable = false;
    bool rearDrivingEnable = false;
    bool leftSteeringEnable = false;
    bool rightSteeringEnable = false;
    bool rearSteeringEnable = false;

    bool tractionEnable = false;
    bool directionEnable = false;

    inLeftDrivingEnable.readNewest(leftDrivingEnable);
    inRightDrivingEnable.readNewest(rightDrivingEnable);
    inRearDrivingEnable.readNewest(rearDrivingEnable);
    inLeftSteeringEnable.readNewest(leftSteeringEnable);
    inRightSteeringEnable.readNewest(rightSteeringEnable);
    inRearSteeringEnable.readNewest(rearSteeringEnable);

    tractionEnable = leftDrivingEnable && rightDrivingEnable && rearDrivingEnable;
    directionEnable = leftSteeringEnable && rightSteeringEnable && rearSteeringEnable;

    outDrivingEnable.write(tractionEnable);
    outSteeringEnable.write(directionEnable);
    outEnable.write( tractionEnable && directionEnable );
}


//-----------------------------------------------------

bool PowerManager::enableTraction()
{
    if( m_tractionEnableOperationInProgress )
    {
        LOGS(Info) << "Can't Enable Traction, an operation is already in progress on traction" << endlog();
        return false;
    }

    m_tractionEnableOperationInProgress = true;
    LOGS(Info) << "Enable Traction required" << endlog();

    if( m_ooEnableLeftDriving.ready() )
        m_ooEnableLeftDriving();
    if( m_ooEnableRightDriving.ready() )
        m_ooEnableRightDriving();
    if( m_ooEnableRearDriving.ready() )
        m_ooEnableRearDriving();

    return true;
}

bool PowerManager::enableDirection()
{
    if( m_directionEnableOperationInProgress )
    {
        LOGS(Info) << "Can't Enable Direction, an operation is already in progress on direction" << endlog();
        return false;
    }

    m_directionEnableOperationInProgress = true;
    LOGS(Info) << "Enable Direction required" << endlog();

    if( m_ooEnableLeftSteering.ready() )
        m_ooEnableLeftSteering();
    if( m_ooEnableRightSteering.ready() )
        m_ooEnableRightSteering();
    if( m_ooEnableRearSteering.ready() )
        m_ooEnableRearSteering();

    return true;
}

bool PowerManager::enablePower()
{
    bool res = true;

    res &= enableTraction();
    res &= enableDirection();

    return true;
}

bool PowerManager::disableTraction()
{
    if( m_tractionDisableOperationInProgress )
    {
        LOGS(Info) << "Can't Disable Traction, an operation is already in progress on traction" << endlog();
        return false;
    }

    m_tractionDisableOperationInProgress = true;
    LOGS(Info) << "Disable Traction required" << endlog();

    if( m_ooDisableLeftDriving.ready() )
        m_ooDisableLeftDriving();
    if( m_ooDisableRightDriving.ready() )
        m_ooDisableRightDriving();
    if( m_ooDisableRearDriving.ready() )
        m_ooDisableRearDriving();

    return true;
}

bool PowerManager::disableDirection()
{
    if( m_directionDisableOperationInProgress )
    {
        LOGS(Info) << "Can't Disable Direction, an operation is already in progress on direction" << endlog();
        return false;
    }

    m_directionDisableOperationInProgress = true;
    LOGS(Info) << "Disable Direction required" << endlog();

    if( m_ooDisableLeftSteering.ready() )
        m_ooDisableLeftSteering();
    if( m_ooDisableRightSteering.ready() )
        m_ooDisableRightSteering();
    if( m_ooDisableRearSteering.ready() )
        m_ooDisableRearSteering();

    return true;
}

bool PowerManager::disablePower()
{
    bool res = true;

    res &= disableTraction();
    res &= disableDirection();

    return true;
}


