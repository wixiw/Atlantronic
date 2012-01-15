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
    propCanRequestTimeout(2.0),
    m_owner(owner)
{
    m_owner.addProperty("propRequireCompleteHardware", propRequireCompleteHardware)
        .doc("Decide weather complete hardware must be present or not");

    m_owner.addProperty("propCanRequestTimeout", propCanRequestTimeout)
        .doc("Timeout when sending a command on the CAN, in s");

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

    m_owner.addOperation("ooSetDrivingMotorPower", &PowerManager::ooSetDrivingMotorPower, this, OwnThread)
            .doc("")
            .arg("","");
    m_owner.addOperation("ooSetSteeringMotorPower", &PowerManager::ooSetSteeringMotorPower, this, OwnThread)
            .doc("")
            .arg("","");
    m_owner.addOperation("ooSetMotorPower", &PowerManager::ooSetMotorPower, this, OwnThread)
            .doc("")
            .arg("","");

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

    //we test the peer existence, because in some cases we don't care in case of incomplete hardware
    if( m_owner.hasPeer("LeftDriving") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("LeftDriving",      "ooEnableDrive",        m_ooEnableLeftDriving);
        res &= m_owner.getOperation("LeftDriving",      "ooDisableDrive",       m_ooDisableLeftDriving);
        res &= m_owner.getOperation("LeftDriving" ,     "ooSetOperationMode",   m_ooSetLeftDrivingOperationMode);
    }
    if( m_owner.hasPeer("RightDriving") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("RightDriving",     "ooEnableDrive",        m_ooEnableRightDriving);
        res &= m_owner.getOperation("RightDriving",     "ooDisableDrive",       m_ooDisableRightDriving);
        res &= m_owner.getOperation("RightDriving",     "ooSetOperationMode",   m_ooSetRightDrivingOperationMode);
    }
    if( m_owner.hasPeer("RearDriving") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("RearDriving",      "ooEnableDrive",        m_ooEnableRearDriving);
        res &= m_owner.getOperation("RearDriving",      "ooDisableDrive",       m_ooDisableRearDriving);
        res &= m_owner.getOperation("RearDriving",      "ooSetOperationMode",   m_ooSetRearDrivingOperationMode);
    }
    if( m_owner.hasPeer("LeftSteering") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("LeftSteering",     "ooEnableDrive",        m_ooEnableLeftSteering);
        res &= m_owner.getOperation("LeftSteering",     "ooDisableDrive",       m_ooDisableLeftSteering);
        res &= m_owner.getOperation("LeftSteering" ,    "ooSetOperationMode",   m_ooSetLeftSteeringOperationMode);
    }
    if( m_owner.hasPeer("RightSteering") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("RightSteering",    "ooEnableDrive",        m_ooEnableRightSteering);
        res &= m_owner.getOperation("RightSteering",    "ooDisableDrive",       m_ooDisableRightSteering);
        res &= m_owner.getOperation("RightSteering",    "ooSetOperationMode",   m_ooSetRightSteeringOperationMode);
    }
    if( m_owner.hasPeer("RearSteering") || propRequireCompleteHardware )
    {
        res &= m_owner.getOperation("RearSteering",     "ooEnableDrive",        m_ooEnableRearSteering);
        res &= m_owner.getOperation("RearSteering",     "ooDisableDrive",       m_ooDisableRearSteering);
        res &= m_owner.getOperation("RearSteering",     "ooSetOperationMode",   m_ooSetRearSteeringOperationMode);
    }

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

    inLeftDrivingEnable.readNewest(leftDrivingEnable);
    inRightDrivingEnable.readNewest(rightDrivingEnable);
    inRearDrivingEnable.readNewest(rearDrivingEnable);
    inLeftSteeringEnable.readNewest(leftSteeringEnable);
    inRightSteeringEnable.readNewest(rightSteeringEnable);
    inRearSteeringEnable.readNewest(rearSteeringEnable);

    outDrivingEnable.write( leftDrivingEnable && rightDrivingEnable && rearDrivingEnable );
    outSteeringEnable.write( leftSteeringEnable && rightSteeringEnable && rearSteeringEnable );
    outEnable.write( outDrivingEnable.getLastWrittenValue() && outSteeringEnable.getLastWrittenValue() );
}

//-----------------------------------------------------

bool PowerManager::ooSetDrivingMotorPower(bool powerOn)
{
    double chrono = 0.0;
    bool leftDrivingEnableTmp;
    bool rightDrivingEnableTmp;
    bool rearDrivingEnableTmp;
    bool enable = outDrivingEnable.getLastWrittenValue();

    //Envoit de la commande d'enable
    if( powerOn )
    {
        if( enable )
        {
            LOGS(Info) << "ooSetDrivingMotorPower : you are trying to power the drive but they are already powered !" << endlog();
        }
        else
        {
            m_ooEnableLeftDriving();
            m_ooEnableRightDriving();
            m_ooEnableRearDriving();
        }
    }
    else
    {
        if( enable )
        {
            m_ooDisableLeftDriving();
            m_ooDisableRightDriving();
            m_ooDisableRearDriving();
        }
        else
        {
            LOGS(Info) << "ooSetDrivingMotorPower : you are trying to unpower the drive but they are already unpowered !" << endlog();
        }
    }

    //Attente de confirmation de l'action :
    whileTimeout(inLeftDrivingEnable.readNewest(leftDrivingEnableTmp) == NoData && leftDrivingEnableTmp!=powerOn
            && inRightDrivingEnable.readNewest(rightDrivingEnableTmp) == NoData && rightDrivingEnableTmp!=powerOn
            && inRearDrivingEnable.readNewest(rearDrivingEnableTmp) == NoData && rearDrivingEnableTmp!=powerOn
            , propCanRequestTimeout, 0.050);
    IfWhileTimeoutExpired(propCanRequestTimeout)
    {
        LOGS(Error) << "ooSetDrivingMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    //remise ne mode vitesse des moteurs
    if( m_ooSetLeftDrivingOperationMode("speed") == false
     || m_ooSetRightDrivingOperationMode("speed") == false
     || m_ooSetRearDrivingOperationMode("speed") == false
     )
    {
        LOGS(Error) << "ooSetDrivingMotorPower : could not switch back to speed mode." << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetDrivingMotorPower : Motors power switched to " << powerOn << " properly." << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool PowerManager::ooSetSteeringMotorPower(bool powerOn)
{
    double chrono = 0.0;
    bool leftSteeringEnableTmp;
    bool rightSteeringEnableTmp;
    bool rearSteeringEnableTmp;
    bool enable = outSteeringEnable.getLastWrittenValue();

    //Envoit de la commande d'enable
    if( powerOn )
    {
        if( enable )
        {
            LOGS(Info) << "ooSetSteeringMotorPower : you are trying to power the drive but they are already powered !" << endlog();
        }
        else
        {
            m_ooEnableLeftSteering();
            m_ooEnableRightSteering();
            m_ooEnableRearSteering();
        }
    }
    else
    {
        if( enable )
        {
            m_ooDisableLeftSteering();
            m_ooDisableRightSteering();
            m_ooDisableRearSteering();
        }
        else
        {
            LOGS(Info) << "ooSetSteeringMotorPower : you are trying to unpower the drive but they are already unpowered !" << endlog();
        }
    }

    //Attente de confirmation de l'action :
    whileTimeout(inLeftSteeringEnable.readNewest(leftSteeringEnableTmp) == NoData && leftSteeringEnableTmp!=powerOn
            && inRightSteeringEnable.readNewest(rightSteeringEnableTmp) == NoData && rightSteeringEnableTmp!=powerOn
            && inRearSteeringEnable.readNewest(rearSteeringEnableTmp) == NoData && rearSteeringEnableTmp!=powerOn
            , propCanRequestTimeout, 0.050);
    IfWhileTimeoutExpired(propCanRequestTimeout)
    {
        LOGS(Error) << "ooSetSteeringMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    //remise ne mode position des moteurs
    if( m_ooSetLeftSteeringOperationMode("position") == false
    || m_ooSetRightSteeringOperationMode("position") == false
    || m_ooSetRearSteeringOperationMode("position") == false
    )
    {
        LOGS(Error) << "ooSetSteeringMotorPower : could not switch back to position mode." << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetSteeringMotorPower : Motors power switched to " << powerOn << " properly." << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}


bool PowerManager::ooSetMotorPower(bool powerOn)
{
    bool res = true;

    res &= ooSetDrivingMotorPower( powerOn );
    res &= ooSetSteeringMotorPower( powerOn );

    return res;
}
