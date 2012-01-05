/*
 * UbiquityItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "UbiquityItf.hpp"
#include "arp_hml_version.h"

#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std_msgs;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::UbiquityItf )

UbiquityItf::UbiquityItf(const std::string& name):
	HmlTaskContext(name),
    attrCurrentCmd(),
    attrOdometers(),
    propSpeedCmdMaxDelay(1.000),
    m_receivedPartialPosition(false)
{
    addAttribute("attrCurrentCmd", attrCurrentCmd);
    addAttribute("attrOdometers", attrOdometers);

    addProperty("propSpeedCmdMaxDelay", propSpeedCmdMaxDelay)
    	.doc("Maximal delay beetween 2 received Differential commands. If this delay is overrun, a speed of 0 is sent on each motor. In s ");

    /** Interface with OUTSIDE (master, ODS, RLU) **/
    addEventPort("inOmniCmd",inOmniCmd)
            .doc("Speed command for the 6 motors motor");
    addPort("outOdometryMeasures",outOdometryMeasures)
        .doc("Odometers value from the motors assembled in an 'Odo' Ros message");
    addPort("outOmniSpeedMeasure",outOmniSpeedMeasure)
    	.doc("Speed measures for left and right motor");
    addPort("outIoStart",outIoStart)
        .doc("Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outEmergencyStop",outEmergencyStop)
        .doc("Is true when HML thinks the emergency stop button is active");
    addPort("outDriveEnable",outDriveEnable)
    	.doc("Is true when the 2 drives are enabled. Since this port is false, drive speed are forced to 0");
    addPort("outWheelBlocked",outWheelBlocked)
         .doc("Is true when wheel are blocked");

    /** Interface with INSIDE (hml !) **/
    addEventPort("inIoStart",inIoStart)
            .doc("HW value of the start switch. It is true when the start is in");

    addEventPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("Value of the left driving odometer in rad on the wheel axe");
    addPort("inLeftDrivingPositionTime",inLeftDrivingPositionTime)
            .doc("");
    addEventPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("Value of the right driving odometer in rad on the wheel axe");
    addPort("inRightDrivingPositionTime",inRightDrivingPositionTime)
            .doc("");
    addEventPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("Value of the rear driving odometer in rad on the wheel axe");
    addPort("inRearDrivingPositionTime",inRearDrivingPositionTime)
            .doc("");
    addEventPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("Value of the left steering odometer in rad on the wheel axe");
    addPort("inLeftSteeringPositionTime",inLeftSteeringPositionTime)
            .doc("");
    addEventPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("Value of the right steering odometer in rad on the wheel axe");
    addPort("inRightSteeringPositionTime",inRightSteeringPositionTime)
            .doc("");
    addEventPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("Value of the rear steering odometer in rad on the wheel axe");
    addPort("inRearSteeringPositionTime",inRearSteeringPositionTime)
            .doc("");

    addPort("inLeftDrivingSpeedMeasure",inLeftDrivingSpeedMeasure)
            .doc("Value of the left driving speed in rad/s on the wheel axe");
    addPort("inRightDrivingSpeedMeasure",inRightDrivingSpeedMeasure)
            .doc("Value of the right driving speed in rad/s on the wheel axe");
    addPort("inRearDrivingSpeedMeasure",inRearDrivingSpeedMeasure)
            .doc("Value of the rear driving speed in rad/s on the wheel axe");
    addPort("inLeftSteeringSpeedMeasure",inLeftSteeringSpeedMeasure)
            .doc("Value of the left steering speed in rad/s on the wheel axe");
    addPort("inRightSteeringSpeedMeasure",inRightSteeringSpeedMeasure)
            .doc("Value of the right steering speed in rad/s on the wheel axe");
    addPort("inRearSteeringSpeedMeasure",inRearSteeringSpeedMeasure)
            .doc("Value of the rear steering speed in rad/s on the wheel axe");

    addPort("inLeftDrivingConnected",inLeftDrivingConnected)
            .doc("Left driveing connectivity");
    addPort("inRightDrivingConnected",inRightDrivingConnected)
             .doc("Right driving connectivity");
    addPort("inRearDrivingConnected",inRearDrivingConnected)
             .doc("Rear driving connectivity");
    addPort("inLeftSteeringConnected",inLeftSteeringConnected)
            .doc("Left steering connectivity");
    addPort("inRightSteeringConnected",inRightSteeringConnected)
             .doc("Right steering connectivity");
    addPort("inRearSteeringConnected",inRearSteeringConnected)
             .doc("Rear steering connectivity");
    addPort("inWoodheadInConnected",inWoodheadInConnected)
             .doc("Input Woodhead connectivity");
    addPort("inWoodheadOutConnected",inWoodheadOutConnected)
             .doc("Output Woodhead connectivity");

    addPort("inLeftDrivingEnable",inLeftDrivingEnable)
            .doc("Left driving soft enable state");
    addPort("inRightDrivingEnable",inRightDrivingEnable)
            .doc("Right driving soft enable state");
    addPort("inRearDrivingEnable",inRearDrivingEnable)
            .doc("Rear driving soft enable state");
    addPort("inLeftSteeringEnable",inLeftSteeringEnable)
            .doc("Left steering soft enable state");
    addPort("inRightSteeringEnable",inRightSteeringEnable)
            .doc("Right steering soft enable state");
    addPort("inRearSteeringEnable",inRearSteeringEnable)
            .doc("Rear steering soft enable state");

    addPort("inLeftDrivingBlocked",inLeftDrivingBlocked)
            .doc("Left driving motor is blocked");
    addPort("inRightDrivingBlocked",inRightDrivingBlocked)
            .doc("Right driving motor is blocked");
    addPort("inRearDrivingBlocked",inRearDrivingBlocked)
            .doc("Rear driving motor is blocked");
    addPort("inLeftSteeringBlocked",inLeftSteeringBlocked)
            .doc("Left steering motor is blocked");
    addPort("inRightSteeringBlocked",inRightSteeringBlocked)
            .doc("Right steering motor is blocked");
    addPort("inRearSteeringBlocked",inRearSteeringBlocked)
            .doc("Rear steering motor is blocked");

    addPort("outLeftDrivingSpeedCmd",outLeftDrivingSpeedCmd)
            .doc("Speed command for the left driving motor in rad/s on the reductor output");
    addPort("outRightDrivingSpeedCmd",outRightDrivingSpeedCmd)
            .doc("Speed command for the right driving motor in rad/s on the reductor output");
    addPort("outRearDrivingSpeedCmd",outRearDrivingSpeedCmd)
            .doc("Speed command for the rear driving motor in rad/s on the reductor output");

    addPort("outLeftSteeringPositionCmd",outLeftSteeringPositionCmd)
            .doc("Position command for the left steering motor in rad/s on the reductor output");
    addPort("outRightSteeringPositionCmd",outRightSteeringPositionCmd)
            .doc("Position command for the right steering motor in rad/s on the reductor output");
    addPort("outRearSteeringPositionCmd",outRearSteeringPositionCmd)
            .doc("Position command for the rear steering motor in rad/s on the reductor output");

    addPort("outLeftDrivingTorqueCmd",outLeftDrivingTorqueCmd)
        .doc("Torque command for the left driving motor in Nm on the reductor output");
    addPort("outRightDrivingTorqueCmd",outRightDrivingTorqueCmd)
        .doc("Torque command for the right driving motor in Nm on the reductor output");
    addPort("outRearDrivingTorqueCmd",outRearDrivingTorqueCmd)
        .doc("Torque command for the rear driving motor in Nm on the reductor output");
    addPort("outLeftSteeringTorqueCmd",outLeftSteeringTorqueCmd)
        .doc("Torque command for the left steering motor in Nm on the reductor output");
    addPort("outRightSteeringTorqueCmd",outRightSteeringTorqueCmd)
        .doc("Torque command for the right steering motor in Nm on the reductor output");
    addPort("outRearSteeringTorqueCmd",outRearSteeringTorqueCmd)
        .doc("Torque command for the rear steering motor in Nm on the reductor output");

    addOperation("coGetCoreVersion",&UbiquityItf::coGetCoreVersion, this, ClientThread)
    		.doc("Returns a string containing Core version");
    addOperation("coGetHmlVersion",&UbiquityItf::coGetHmlVersion, this, ClientThread)
    		.doc("Returns a string containing HML version");
    addOperation("ooSetMotorPower",&UbiquityItf::ooSetMotorPower, this, OwnThread)
            .doc("Defines if motor are powered or not. Returns true if succeed")
            .arg("powerOn","when set to true motor is powered and ready to move. If a null speed is provided, the motor is locked as if brakes where enabled. When set to false there is no power in the motor which means that the motor is free to any move.")
            .arg("timeout","maximal time to wait for completion");
    addOperation("ooResetHml",&UbiquityItf::ooResetHml, this, OwnThread)
    	.doc("Ask all cane node to reset. Could be usefull after an emergency stop");

    ros::NodeHandle nh;
    m_srvSetMotorPower = nh.advertiseService("/Protokrot/setMotorPower", &UbiquityItf::srvSetMotorPower, this);
    m_srvResetHml = nh.advertiseService("/Protokrot/resetHml", &UbiquityItf::srvResetHml, this);
}

string UbiquityItf::coGetCoreVersion()
{
	return "not implemented yet";
}

string UbiquityItf::coGetHmlVersion()
{
	return ARP_HML_VERSION;
}

bool UbiquityItf::configureHook()
{
    bool res = true;
    HmlTaskContext::configureHook();
    res &= getOperation("LeftDriving" ,  "ooEnableDrive",       m_ooEnableLeftDriving);
    res &= getOperation("RightDriving", "ooEnableDrive",        m_ooEnableRigthtDriving);
    res &= getOperation("RearDriving" ,  "ooEnableDrive",       m_ooEnableRearDriving);
    res &= getOperation("LeftSteering" ,  "ooEnableDrive",      m_ooEnableLeftSteering);
    res &= getOperation("RightSteering", "ooEnableDrive",       m_ooEnableRigthtSteering);
    res &= getOperation("RearSteering" ,  "ooEnableDrive",      m_ooEnableRearSteering);

    res &= getOperation("LeftDriving" ,  "ooDisableDrive",      m_ooDisableLeftDriving);
    res &= getOperation("RightDriving", "ooDisableDrive",       m_ooDisableRightDriving);
    res &= getOperation("RearDriving", "ooDisableDrive",        m_ooDisableRearDriving);
    res &= getOperation("LeftSteering" ,  "ooDisableDrive",     m_ooDisableLeftSteering);
    res &= getOperation("RightSteering", "ooDisableDrive",      m_ooDisableRightSteering);
    res &= getOperation("RearSteering", "ooDisableDrive",       m_ooDisableRearSteering);

    res &= getOperation("LeftDriving" ,  "ooSetOperationMode",  m_ooSetLeftDrivingOperationMode);
    res &= getOperation("RightDriving", "ooSetOperationMode",  m_ooSetRightDrivingOperationMode);
    res &= getOperation("RearDriving", "ooSetOperationMode",  m_ooSetRearDrivingOperationMode);
    res &= getOperation("LeftSteering" ,  "ooSetOperationMode",  m_ooSetLeftSteeringOperationMode);
    res &= getOperation("RightSteering", "ooSetOperationMode",  m_ooSetRightSteeringOperationMode);
    res &= getOperation("RearSteering", "ooSetOperationMode",  m_ooSetRearSteeringOperationMode);

    res &= getOperation("WoodheadOut"    , "coReset",  m_coResetWoodheadOut);
    res &= getOperation("WoodheadIn"    , "coReset",  m_coResetWoodheadIn);
    res &= getOperation("LeftDriving"  , "coReset",  m_coResetLeftDriving);
    res &= getOperation("RightDriving" , "coReset",  m_coResetRightDriving);
    res &= getOperation("RearDriving" , "coReset",  m_coResetRearDriving);
    res &= getOperation("LeftSteering"  , "coReset",  m_coResetLeftSteering);
    res &= getOperation("RightSteering" , "coReset",  m_coResetRightSteering);
    res &= getOperation("RearSteering" , "coReset",  m_coResetRearSteering);

    if( res == false )
    {
        LOG(Error) << "failed to configure : did not get operations" << endlog();
    }

    return res;
}

void UbiquityItf::updateHook()
{
	HmlTaskContext::updateHook();

	//publication des commandes de vitesse
	writeOmniCmd();

    //lecture des valeurs des odomètres
    readOdometers();

    //lecture du start
    readStart();

    //lecture de enable
    readDriveEnable();

    //lecture de la conenctivité
    readConnectivity();

    //lecture des vitesses
    readSpeed();

    //lecture du blocage roues
    readWheelBlocked();
}

void UbiquityItf::writeOmniCmd()
{
    OmniCommand cmd;
	struct timespec now;
	double delayInS;

    if(NewData==inOmniCmd.read(cmd))
    {
    	//ecriture des consignes moteurs
        Bool enable = outDriveEnable.getLastWrittenValue();
    	if( enable.data )
    	{
			clock_gettime(CLOCK_MONOTONIC, &m_lastCmdTimestamp);
			attrCurrentCmd = cmd;
    	}
    	//si les moteurs sont disable on n'envoit pas de consigne
    	else
    	{
            attrCurrentCmd.v_left_driving = 0;
            attrCurrentCmd.v_right_driving = 0;
            attrCurrentCmd.v_rear_driving = 0;
            attrCurrentCmd.p_left_steering = 0;
            attrCurrentCmd.p_right_steering = 0;
            attrCurrentCmd.p_rear_steering = 0;
    	}
    }
    else
    {
    	//si on n'a pas reçu de commande, au bout d'une seconde on "met les freins"
    	clock_gettime(CLOCK_MONOTONIC, &now);
    	delayInS = delta_t(m_lastCmdTimestamp, now);
    	if( delayInS >= propSpeedCmdMaxDelay )
        {
            attrCurrentCmd.v_left_driving = 0;
            attrCurrentCmd.v_right_driving = 0;
            attrCurrentCmd.v_rear_driving = 0;
            attrCurrentCmd.p_left_steering = 0;
            attrCurrentCmd.p_right_steering = 0;
            attrCurrentCmd.p_rear_steering = 0;
        }
    }

    outLeftDrivingSpeedCmd.write(attrCurrentCmd.v_left_driving);
    outRightDrivingSpeedCmd.write(attrCurrentCmd.v_right_driving);
    outRearDrivingSpeedCmd.write(attrCurrentCmd.v_rear_driving);
    outLeftSteeringPositionCmd.write(attrCurrentCmd.p_left_steering);
    outRightSteeringPositionCmd.write(attrCurrentCmd.p_right_steering);
    outRearSteeringPositionCmd.write(attrCurrentCmd.p_rear_steering);
}

void UbiquityItf::readOdometers()
{
    double odoValueLeftDriving;
    double odoValueRightDriving;
    double odoValueRearDriving;
    double odoValueLeftSteering;
    double odoValueRightSteering;
    double odoValueRearSteering;

    double odoTimeLeftDriving;
    double odoTimeRightDriving;
    double odoTimeRearDriving;
    double odoTimeLeftSteering;
    double odoTimeRightSteering;
    double odoTimeRearSteering;

    if(         NoData != inLeftDrivingPositionTime.readNewest(odoTimeLeftDriving)
            &&  NoData != inRightDrivingPositionTime.readNewest(odoTimeRightDriving)
            &&  NoData != inRearDrivingPositionTime.readNewest(odoTimeRearDriving)
            &&  NoData != inLeftSteeringPositionTime.readNewest(odoTimeLeftSteering)
            &&  NoData != inRightSteeringPositionTime.readNewest(odoTimeRightSteering)
            &&  NoData != inRearSteeringPositionTime.readNewest(odoTimeRearSteering)
    )
    {
    	if(         odoTimeLeftDriving != odoTimeRightDriving
    	        ||  odoTimeRightDriving != odoTimeRearDriving
    	        ||  odoTimeRearDriving != odoTimeLeftSteering
    	        ||  odoTimeLeftSteering != odoTimeRightSteering
                ||  odoTimeRightSteering != odoTimeRearSteering
    	    )
    	{
    	    if( m_receivedPartialPosition == true )
    	    {
    	        LOG(Error) << "Did not receive the two Odometry measures in time-> Reseting HML" << endlog();
    	        cerr << "Did not receive the two Odometry measures in time-> Reseting HML" << endl;
    	        ooResetHml();
    	        m_receivedPartialPosition = false;
    	        ooSetMotorPower(true,1.0);
    	    }
    	    else
    	    {
    	        m_receivedPartialPosition = true;
    	    }
    	}
    	else
    	{
    	    //on ne reproduit pas le message si c'est le dernier qui a été publié
    	    if( attrOdometers.time != odoTimeLeftDriving )
    	    {
                m_receivedPartialPosition = false;
                inLeftDrivingPosition.readNewest(odoValueLeftDriving);
                inRightDrivingPosition.readNewest(odoValueRightDriving);
                inRearDrivingPosition.readNewest(odoValueRearDriving);
                inLeftSteeringPosition.readNewest(odoValueLeftSteering);
                inRightSteeringPosition.readNewest(odoValueRightSteering);
                inRearSteeringPosition.readNewest(odoValueRearSteering);
                attrOdometers.odo_left_driving = odoValueLeftDriving;
                attrOdometers.odo_right_driving = odoValueRightDriving;
                attrOdometers.odo_rear_driving = odoValueRearDriving;
                attrOdometers.odo_left_steering = odoValueLeftSteering;
                attrOdometers.odo_right_steering = odoValueRightSteering;
                attrOdometers.odo_rear_steering = odoValueRearSteering;
                attrOdometers.time = odoTimeLeftDriving;
                outOdometryMeasures.write(attrOdometers);
    	    }
    	}
    }
}

void UbiquityItf::readStart()
{
	bool io = false;
	Start start;
	if(NewData==inIoStart.readNewest(io))
	{
	    start.go = !io;
	    outIoStart.write(start);
	}
}

void UbiquityItf::readDriveEnable()
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

	Bool enable;
	enable.data = leftDrivingEnable && rightDrivingEnable && rearDrivingEnable
	            && leftSteeringEnable && rightSteeringEnable && rearSteeringEnable;
    outDriveEnable.write( enable );
}

void UbiquityItf::readConnectivity()
{
    bool leftDrivingConnectivity = false;
    bool rightDrivingConnectivity = false;
    bool rearDrivingConnectivity = false;
    bool leftSteeringConnectivity = false;
    bool rightSteeringConnectivity = false;
    bool rearSteeringConnectivity = false;
    bool woodheadOConnectivity = false;
    bool woodheadIConnectivity = false;
    Bool emergency;

    inLeftDrivingConnected.readNewest(leftDrivingConnectivity);
    inRightDrivingConnected.readNewest(rightDrivingConnectivity);
    inRearDrivingConnected.readNewest(rearDrivingConnectivity);
    inLeftSteeringConnected.readNewest(leftSteeringConnectivity);
    inRightSteeringConnected.readNewest(rightSteeringConnectivity);
    inRearSteeringConnected.readNewest(rearSteeringConnectivity);
    inWoodheadOutConnected.readNewest(woodheadOConnectivity);
    inWoodheadInConnected.readNewest(woodheadIConnectivity);

    emergency.data = !leftDrivingConnectivity && !rightDrivingConnectivity && !rearDrivingConnectivity
                    && !leftSteeringConnectivity && !rightSteeringConnectivity && !rearSteeringConnectivity
                    && !woodheadOConnectivity && !woodheadIConnectivity;
    outEmergencyStop.write(emergency);
}

void UbiquityItf::readSpeed()
{
	double leftDrivingSpeed = 0.0;
	double rightDrivingSpeed = 0.0;
	double rearDrivingSpeed = 0.0;

    double leftSteeringPosition = 0.0;
    double rightSteeringPosition = 0.0;
    double rearSteeringPosition = 0.0;

	OmniCommand speedMeasure;
	if( NoData != inLeftDrivingSpeedMeasure.readNewest(leftDrivingSpeed)
	 && NoData != inRightDrivingSpeedMeasure.readNewest(rightDrivingSpeed)
	 && NoData != inRearDrivingSpeedMeasure.readNewest(rearDrivingSpeed)

	 && NoData != inLeftSteeringPosition.readNewest(leftSteeringPosition)
     && NoData != inRightSteeringPosition.readNewest(rightSteeringPosition)
     && NoData != inRearSteeringPosition.readNewest(rearSteeringPosition)
	)
	{
		speedMeasure.v_left_driving = leftDrivingSpeed;
		speedMeasure.v_right_driving = rightDrivingSpeed;
		speedMeasure.v_rear_driving = rearDrivingSpeed;

        speedMeasure.p_left_steering = leftSteeringPosition;
        speedMeasure.p_right_steering = rightSteeringPosition;
        speedMeasure.p_rear_steering = rearSteeringPosition;
		outOmniSpeedMeasure.write(speedMeasure);
	}
}

void UbiquityItf::readWheelBlocked()
{
    bool leftWheelBlocked = false;
    bool rightWheelBlocked = false;
    bool rearWheelBlocked = false;

    Bool blocked;
    inLeftDrivingBlocked.readNewest(leftWheelBlocked);
    inRightDrivingBlocked.readNewest(rightWheelBlocked);
    inRearDrivingBlocked.readNewest(rearWheelBlocked);
    blocked.data = leftWheelBlocked || rightWheelBlocked || rearWheelBlocked;
    outWheelBlocked.write(blocked);
}



bool UbiquityItf::ooSetMotorPower(bool powerOn, double timeout)
{
    double chrono = 0.0;
    bool leftDrivingEnableTmp;
    bool rightDrivingEnableTmp;
    bool rearDrivingEnableTmp;
    bool leftSteeringEnableTmp;
    bool rightSteeringEnableTmp;
    bool rearSteeringEnableTmp;
    Bool enable = outDriveEnable.getLastWrittenValue();

    //Envoit de la commande d'enable
    if( powerOn )
    {
        if( enable.data )
        {
            LOG(Info) << "ooMotorPower : you are trying to power the drive but they are already powered !" << endlog();
        }
        else
        {
            m_ooEnableLeftDriving();
            m_ooEnableRigthtDriving();
            m_ooEnableRearDriving();
            m_ooEnableLeftSteering();
            m_ooEnableRigthtSteering();
            m_ooEnableRearSteering();
        }
    }
    else
    {
        if( enable.data )
        {
            m_ooDisableLeftDriving();
            m_ooDisableRightDriving();
            m_ooDisableRearDriving();
            m_ooDisableLeftSteering();
            m_ooDisableRightSteering();
            m_ooDisableRearSteering();
        }
        else
        {
            LOG(Info) << "ooMotorPower : you are trying to unpower the drive but they are already unpowered !" << endlog();
        }
    }

    //Attente de confirmation de l'action :
    whileTimeout(inLeftDrivingEnable.readNewest(leftDrivingEnableTmp) != NoData && leftDrivingEnableTmp!=powerOn
            && inRightDrivingEnable.readNewest(rightDrivingEnableTmp) != NoData && rightDrivingEnableTmp!=powerOn
            && inRearDrivingEnable.readNewest(rearDrivingEnableTmp) != NoData && rearDrivingEnableTmp!=powerOn
            && inLeftSteeringEnable.readNewest(leftSteeringEnableTmp) != NoData && leftSteeringEnableTmp!=powerOn
            && inRightSteeringEnable.readNewest(rightSteeringEnableTmp) != NoData && rightSteeringEnableTmp!=powerOn
            && inRearSteeringEnable.readNewest(rearSteeringEnableTmp) != NoData && rearSteeringEnableTmp!=powerOn
            , timeout, 0.050);
    IfWhileTimeoutExpired(timeout)
    {
        LOG(Error) << "ooMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    //remise ne mode vitesse des moteurs
    if( m_ooSetLeftDrivingOperationMode("speed") == false
     || m_ooSetRightDrivingOperationMode("speed") == false
     || m_ooSetRearDrivingOperationMode("speed") == false
     )
    {
        LOG(Error) << "ooMotorPower : could not switch back to speed mode." << endlog();
        goto failed;
    }

    if( m_ooSetLeftSteeringOperationMode("position") == false
    || m_ooSetRightSteeringOperationMode("position") == false
    || m_ooSetRearSteeringOperationMode("position") == false
    )
    {
        LOG(Error) << "ooMotorPower : could not switch back to position mode." << endlog();
        goto failed;
    }

    LOG(Info) << "ooMotorPower : Motors power switched to " << powerOn << " properly." << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool UbiquityItf::srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = ooSetMotorPower(req.powerOn, req.timeout);
    return true;
}

bool UbiquityItf::ooResetHml()
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

bool UbiquityItf::srvResetHml(ResetHml::Request& req, ResetHml::Response& res)
{
    res.success = ooResetHml();
    return true;
}

