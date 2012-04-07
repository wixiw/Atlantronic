/*
 * PowerManager.cpp
 *
 *  Created on: 7 janv. 2012
 *      Author: ard, wla
 */
#include "PowerManager.hpp"

using namespace arp_hml;
using namespace arp_core;
using namespace std;
using namespace RTT;

PowerManager::PowerManager(ARDTaskContext& owner):
    propCanRequestTimeout(2.0),
    m_owner(owner)
{
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


    m_owner.addPort("inLeftDrivingConnected",inLeftDrivingConnected)
            .doc("Left driveing connectivity");
    m_owner.addPort("inRightDrivingConnected",inRightDrivingConnected)
             .doc("Right driving connectivity");
    m_owner.addPort("inRearDrivingConnected",inRearDrivingConnected)
             .doc("Rear driving connectivity");
    m_owner.addPort("inLeftSteeringConnected",inLeftSteeringConnected)
            .doc("Left steering connectivity");
    m_owner.addPort("inRightSteeringConnected",inRightSteeringConnected)
             .doc("Right steering connectivity");
    m_owner.addPort("inRearSteeringConnected",inRearSteeringConnected)
             .doc("Rear steering connectivity");
    m_owner.addPort("inWoodheadInConnected",inWoodheadInConnected)
             .doc("Input Woodhead connectivity");
    m_owner.addPort("inWoodheadOutConnected",inWoodheadOutConnected)
             .doc("Output Woodhead connectivity");

    m_owner.addPort("outDrivingEnable",outDrivingEnable)
            .doc("Driving soft enable state");
    m_owner.addPort("outSteeringEnable",outSteeringEnable)
            .doc("Steering soft enable state");
    m_owner.addPort("outEnable",outEnable)
            .doc("All device are soft enabled");

    m_owner.addOperation("coSetDrivingMotorPower", &PowerManager::coSetDrivingMotorPower, this, ClientThread)
            .doc("")
            .arg("","");
    m_owner.addOperation("coSetSteeringMotorPower", &PowerManager::coSetSteeringMotorPower, this, ClientThread)
            .doc("")
            .arg("","");
    m_owner.addOperation("coSetMotorPower", &PowerManager::coSetMotorPower, this, ClientThread)
            .doc("")
            .arg("","");

}

PowerManager::~PowerManager()
{

}

//------------------------------------------------------------------------------------------------------------------

bool PowerManager::configureHook()
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
        //get ppers operations to show a simplier interface to the outside world
        res &= getPeersOperations();
    }

    return res;
}

bool PowerManager::getPeersOperations()
{
    bool res = true;

    //we test the peer existence, because in some cases we don't care in case of incomplete hardware
    if( m_owner.hasPeer("LeftDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("LeftDriving",      "ooEnableDrive",        m_ooEnableLeftDriving);
        res &= m_owner.getOperation("LeftDriving",      "ooDisableDrive",       m_ooDisableLeftDriving);
    }
    if( m_owner.hasPeer("RightDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RightDriving",     "ooEnableDrive",        m_ooEnableRightDriving);
        res &= m_owner.getOperation("RightDriving",     "ooDisableDrive",       m_ooDisableRightDriving);
    }
    if( m_owner.hasPeer("RearDriving") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RearDriving",      "ooEnableDrive",        m_ooEnableRearDriving);
        res &= m_owner.getOperation("RearDriving",      "ooDisableDrive",       m_ooDisableRearDriving);
    }
    if( m_owner.hasPeer("LeftSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("LeftSteering",     "ooEnableDrive",        m_ooEnableLeftSteering);
        res &= m_owner.getOperation("LeftSteering",     "ooDisableDrive",       m_ooDisableLeftSteering);
    }
    if( m_owner.hasPeer("RightSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RightSteering",    "ooEnableDrive",        m_ooEnableRightSteering);
        res &= m_owner.getOperation("RightSteering",    "ooDisableDrive",       m_ooDisableRightSteering);
    }
    if( m_owner.hasPeer("RearSteering") || m_propRequireCompleteHardware->get() )
    {
        res &= m_owner.getOperation("RearSteering",     "ooEnableDrive",        m_ooEnableRearSteering);
        res &= m_owner.getOperation("RearSteering",     "ooDisableDrive",       m_ooDisableRearSteering);
    }

    res &= m_owner.getOperation("HmlMonitor",     "ooSetDrivingOperationMode",   m_ooSetDrivingOperationMode);
    res &= m_owner.getOperation("HmlMonitor",     "ooSetSteeringOperationMode",   m_ooSetSteeringOperationMode);

    return res;
}

void PowerManager::updateHook()
{
    readDriveEnable();
    readConnectivity();
}

void PowerManager::readDriveEnable()
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

void PowerManager::readConnectivity()
{
    bool leftDrivingConnectivity = false;
    bool rightDrivingConnectivity = false;
    bool rearDrivingConnectivity = false;
    bool leftSteeringConnectivity = false;
    bool rightSteeringConnectivity = false;
    bool rearSteeringConnectivity = false;
    bool woodheadOConnectivity = false;
    bool woodheadIConnectivity = false;
    bool emergency;

    inLeftDrivingConnected.readNewest(leftDrivingConnectivity);
    inRightDrivingConnected.readNewest(rightDrivingConnectivity);
    inRearDrivingConnected.readNewest(rearDrivingConnectivity);
    inLeftSteeringConnected.readNewest(leftSteeringConnectivity);
    inRightSteeringConnected.readNewest(rightSteeringConnectivity);
    inRearSteeringConnected.readNewest(rearSteeringConnectivity);
    inWoodheadOutConnected.readNewest(woodheadOConnectivity);
    inWoodheadInConnected.readNewest(woodheadIConnectivity);

    emergency = !leftDrivingConnectivity && !rightDrivingConnectivity && !rearDrivingConnectivity
                    && !leftSteeringConnectivity && !rightSteeringConnectivity && !rearSteeringConnectivity
                    && !woodheadOConnectivity && !woodheadIConnectivity;
    outEmergencyStop.write(emergency);
}

//-----------------------------------------------------

bool PowerManager::coSetDrivingMotorPower(bool powerOn)
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
            goto success;
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
            goto success;
        }
    }

    chrono = 0.0;
    //Attente de confirmation de l'action :
    whileTimeout(inLeftDrivingEnable.readNewest(leftDrivingEnableTmp) == NoData && leftDrivingEnableTmp!=powerOn
            && inRightDrivingEnable.readNewest(rightDrivingEnableTmp) == NoData && rightDrivingEnableTmp!=powerOn
            && inRearDrivingEnable.readNewest(rearDrivingEnableTmp) == NoData && rearDrivingEnableTmp!=powerOn
            , propCanRequestTimeout, 0.001);
    IfWhileTimeoutExpired(propCanRequestTimeout)
    {
        LOGS(Error) << "ooSetDrivingMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetSteeringMotorPower : returning to speed mode" << endlog();

    //remise en mode vitesse des moteurs
    if( m_ooSetDrivingOperationMode("speed") == false )
    {
        LOGS(Error) << "m_ooSetDrivingOperationMode : could not switch back to speed mode." << endlog();
        goto failed;
    }
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool PowerManager::coSetSteeringMotorPower(bool powerOn)
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

    chrono = 0.0;
    //Attente de confirmation de l'action :
    whileTimeout(inLeftSteeringEnable.readNewest(leftSteeringEnableTmp) == NoData && leftSteeringEnableTmp!=powerOn
            && inRightSteeringEnable.readNewest(rightSteeringEnableTmp) == NoData && rightSteeringEnableTmp!=powerOn
            && inRearSteeringEnable.readNewest(rearSteeringEnableTmp) == NoData && rearSteeringEnableTmp!=powerOn
            , propCanRequestTimeout, 0.001);
    IfWhileTimeoutExpired(propCanRequestTimeout)
    {
        LOGS(Error) << "ooSetSteeringMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    LOGS(Info) << "ooSetSteeringMotorPower : returning to position mode" << endlog();

    //remise ne mode position des moteurs
    if( m_ooSetSteeringOperationMode("position") == false )
    {
        LOGS(Error) << "ooSetSteeringMotorPower : could not switch back to position mode." << endlog();
        goto failed;
    }
    goto success;

    failed:
        return false;
    success:
        return true;
}


bool PowerManager::coSetMotorPower(bool powerOn)
{
    bool res = true;

    res &= coSetDrivingMotorPower( powerOn );
    res &= coSetSteeringMotorPower( powerOn );

    return res;
}
