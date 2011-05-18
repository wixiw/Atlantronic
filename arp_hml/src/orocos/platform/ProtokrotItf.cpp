/*
 * ProtokrotItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "ProtokrotItf.hpp"
#include "arp_hml_version.h"

#include <ocl/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std_msgs;

ORO_LIST_COMPONENT_TYPE( arp_hml::ProtokrotItf )

ProtokrotItf::ProtokrotItf(const std::string& name):
	HmlTaskContext(name),
    attrCurrentCmd(),
    attrOdometers(),
    propLeftOdometerGain(1),
    propRightOdometerGain(1),
    propLeftSpeedGain(1),
    propRightSpeedGain(1),
    propSpeedCmdMaxDelay(1.000),
    m_receivedPartialPosition(false)
{
    addAttribute("attrCurrentCmd", attrCurrentCmd);
    addAttribute("attrOdometers", attrOdometers);

    addProperty("propLeftOdometerGain",propLeftOdometerGain)
    	.doc("Gain on the left odometer to provide a measure in rad on the wheel axe.");
    addProperty("propRightOdometerGain",propRightOdometerGain)
    	.doc("Gain on the right odometer to provide a measure in rad on the wheel axe.");
    addProperty("propLeftSpeedGain",propLeftSpeedGain)
    	.doc("Gain on the left motor's speed to provide a command in RPM (from rad/s on the wheel axe) on the motor axe.");
    addProperty("propRightSpeedGain",propRightSpeedGain)
    	.doc("Gain on the right motor's speed to provide a command in RPM (from rad/s on the wheel axe) on the motor axe.");
    addProperty("propSpeedCmdMaxDelay", propSpeedCmdMaxDelay)
    	.doc("Maximal delay beetween 2 received Differential commands. If this delay is overrun, a speed of 0 is sent on each motor. In s ");

    /** Interface with OUTSIDE (master, ODS, RLU) **/
    addEventPort("inDifferentialCmd",inDifferentialCmd)
            .doc("Speed command for left and right motor");
    addPort("outOdometryMeasures",outOdometryMeasures)
        .doc("Odometers value from left and right wheel assembled in an 'Odo' Ros message");
    addPort("outDifferentialMeasure",outDifferentialMeasure)
    	.doc("Speed measures for left and right motor");
    addPort("outIoStart",outIoStart)
        .doc("Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoColorSwitch",outIoColorSwitch)
        .doc("Value of the color switch. It is 'blue' when the switch button is on the front side of the robot)"
          "'red' when the button is on the rear side of the robot");
    addPort("outEmergencyStop",outEmergencyStop)
        .doc("Is true when HML thinks the emergency stop button is active");
    addPort("outDriveEnable",outDriveEnable)
    	.doc("");

    /** Interface with INSIDE (hml !) **/
    addEventPort("inIoStart",inIoStart)
            .doc("HW value of the start switch. It is true when the start is in");
    addPort("inIoColorSwitch",inIoColorSwitch)
            .doc("HW value of the color switch. It is true when the color switch is on 1");//on n'est pas pressé pour connaitre la couleur
    addEventPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("Value of the left odometer in rad on the wheel axe");
    addPort("inLeftDrivingPositionTime",inLeftDrivingPositionTime)
            .doc("");
    addEventPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("Value of the right odometer in rad on the wheel axe");
    addPort("inRightDrivingPositionTime",inRightDrivingPositionTime)
            .doc("inRightDrivingPositionTime");
    addPort("inLeftSpeedMeasure",inLeftSpeedMeasure)
            .doc("Value of the left speed in rad/s on the wheel axe");
    addPort("inLeftDriveConnected",inLeftDriveConnected)
            .doc("Left drive connectivity");
    addPort("inRightDriveConnected",inRightDriveConnected)
              .doc("Right drive connectivity");
    addPort("inWoodheadConnected",inWoodheadConnected)
              .doc("Woodhead connectivity");
    addPort("inRightSpeedMeasure",inRightSpeedMeasure)
            .doc("Value of the right speed in rad/s on the wheel axe");

    addPort("inLeftDriveEnable",inLeftDriveEnable)
    		.doc("Left drive soft enable state");
    addPort("inRightDriveEnable",inRightDriveEnable)
    		.doc("Right drive soft enable state");
    addPort("outLeftSpeedCmd",outLeftSpeedCmd)
            .doc("Speed command for the left motor in rad/s on the wheel axe");
    addPort("outRightSpeedCmd",outRightSpeedCmd)
            .doc("Speed command for the right motor in rad/s on the wheel axe");

    addOperation("coGetCoreVersion",&ProtokrotItf::coGetCoreVersion, this, ClientThread)
    		.doc("Returns a string containing Core version");
    addOperation("coGetHmlVersion",&ProtokrotItf::coGetHmlVersion, this, ClientThread)
    		.doc("Returns a string containing HML version");
    addOperation("ooSetMotorPower",&ProtokrotItf::ooSetMotorPower, this, OwnThread)
            .doc("Defines if motor are powered or not. Returns true if succeed")
            .arg("powerOn","when set to true motor is powered and ready to move. If a null speed is provided, the motor is locked as if brakes where enabled. When set to false there is no power in the motor which means that the motor is free to any move.")
            .arg("timeout","maximal time to wait for completion");
    addOperation("ooResetHml",&ProtokrotItf::ooResetHml, this, OwnThread)
    	.doc("Ask all cane node to reset. Could be usefull after an emergency stop");

    ros::NodeHandle nh;
    m_srvSetMotorPower = nh.advertiseService("/Protokrot/setMotorPower", &ProtokrotItf::srvSetMotorPower, this);
    m_srvResetHml = nh.advertiseService("/Protokrot/resetHml", &ProtokrotItf::srvResetHml, this);
}

string ProtokrotItf::coGetCoreVersion()
{
	return "not implemented yet";
}

string ProtokrotItf::coGetHmlVersion()
{
	return ARP_HML_VERSION;
}

bool ProtokrotItf::configureHook()
{
    bool res = true;
    HmlTaskContext::configureHook();
    res &= getOperation("LeftDriving" ,  "ooEnableDrive",       m_ooEnableLeftDrive);
    res &= getOperation("RightDriving", "ooEnableDrive",       m_ooEnableRigthtDrive);
    res &= getOperation("LeftDriving" ,  "ooDisableDrive",      m_ooDisableLeftDrive);
    res &= getOperation("RightDriving", "ooDisableDrive",      m_ooDisableRightDrive);
    res &= getOperation("LeftDriving" ,  "ooSetOperationMode",  m_ooSetLeftOperationMode);
    res &= getOperation("RightDriving", "ooSetOperationMode",  m_ooSetRightOperationMode);

    res &= getOperation("Io"           , "coReset",  m_coResetIo);
    res &= getOperation("LeftDriving"  , "coReset",  m_coResetLeftDriving);
    res &= getOperation("RightDriving" , "coReset",  m_coResetRightDriving);

    if( res == false )
    {
        LOG(Error) << "failed to configure : did not get operations" << endlog();
    }

    return res;
}

void ProtokrotItf::updateHook()
{
	HmlTaskContext::updateHook();

	//publication des commandes de vitesse
	writeDifferentialCmd();

    //lecture des valeurs des odomètres
    readOdometers();

    //Lecture de la couleur
    readColorSwitch();

    //lecture du start
    readStart();

    //lecture de enable
    readDriveEnable();

    //lecture de la conenctivité
    readConnectivity();

    //lecture des vitesses
    readSpeed();
}

void ProtokrotItf::writeDifferentialCmd()
{
    DifferentialCommand cmd;
	struct timespec now;
	double delayInS;

    if(NewData==inDifferentialCmd.read(cmd))
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
            attrCurrentCmd.v_left = 0;
            attrCurrentCmd.v_right = 0;
    	}
    }
    else
    {
    	//si on n'a pas reçu de commande, au bout d'une seconde on "met les freins"
    	clock_gettime(CLOCK_MONOTONIC, &now);
    	delayInS = delta_t(m_lastCmdTimestamp, now);
    	if( delayInS >= propSpeedCmdMaxDelay )
        {
            attrCurrentCmd.v_left = 0;
            attrCurrentCmd.v_right = 0;

        }
    }

    outLeftSpeedCmd.write(attrCurrentCmd.v_left*propLeftSpeedGain);
    outRightSpeedCmd.write(attrCurrentCmd.v_right*propRightSpeedGain);
}

void ProtokrotItf::readOdometers()
{
    double odoValueLeft;
    double odoValueRight;
    double odoTimeLeft;
    double odoTimeRight;
    if(NoData != inLeftDrivingPositionTime.readNewest(odoTimeLeft) && NoData != inRightDrivingPositionTime.readNewest(odoTimeRight) )
    {
    	if( odoTimeLeft != odoTimeRight )
    	{
    	    if( m_receivedPartialPosition == true )
    	    {
    	        LOG(Error) << "should not received twice a partial odometer command" << endlog();
    	    }
    	    else
    	    {
    	        m_receivedPartialPosition = true;
    	    }
    	}
    	else
    	{
    	    //on ne reproduit pas le message si c'est le dernier qui a été publié
    	    if( attrOdometers.time != odoTimeLeft )
    	    {
                m_receivedPartialPosition = false;
                inLeftDrivingPosition.readNewest(odoValueLeft);
                inRightDrivingPosition.readNewest(odoValueRight);
                attrOdometers.odo_left = odoValueLeft*propLeftOdometerGain;
                attrOdometers.odo_right = odoValueRight*propRightOdometerGain;
                attrOdometers.time = odoTimeLeft;
                outOdometryMeasures.write(attrOdometers);
    	    }
    	}
    }
}

void ProtokrotItf::readColorSwitch()
{
    bool io = false;
    StartColor colorSwitch;
    if(NewData==inIoColorSwitch.readNewest(io))
    {
    	if(io)
    	    	colorSwitch.color = "red";
		else
			colorSwitch.color = "blue";

		outIoColorSwitch.write(colorSwitch);
    }
    inIoColorSwitch.readNewest(io);
}

void ProtokrotItf::readStart()
{
	bool io = false;
	Start start;
	if(NewData==inIoStart.readNewest(io))
	{
	    start.go = !io;
	    outIoStart.write(start);
	}
}

void ProtokrotItf::readDriveEnable()
{
	bool leftDriveEnable = false;
	bool rightDriveEnable = false;
	inLeftDriveEnable.readNewest(leftDriveEnable);
	inRightDriveEnable.readNewest(rightDriveEnable);

	Bool enable;
	enable.data = leftDriveEnable && rightDriveEnable;
    outDriveEnable.write( enable );
}

void ProtokrotItf::readConnectivity()
{
    bool leftDriveConnectivity = false;
    bool rightDriveConnectivity = false;
    bool woodheadConnectivity = false;
    Bool emergency;

    inLeftDriveConnected.readNewest(leftDriveConnectivity);
    inRightDriveConnected.readNewest(rightDriveConnectivity);
    inWoodheadConnected.readNewest(woodheadConnectivity);

    emergency.data = !leftDriveConnectivity && !rightDriveConnectivity && !woodheadConnectivity;
    outEmergencyStop.write(emergency);
}

void ProtokrotItf::readSpeed()
{
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	DifferentialCommand speedMeasure;
	if( NoData != inLeftSpeedMeasure.readNewest(leftSpeed) && NoData != inRightSpeedMeasure.readNewest(rightSpeed) )
	{
		speedMeasure.v_left = leftSpeed;
		speedMeasure.v_right = rightSpeed;
		outDifferentialMeasure.write(speedMeasure);
	}
}

bool ProtokrotItf::ooSetMotorPower(bool powerOn, double timeout)
{
    double chrono = 0.0;
    bool leftEnableTmp;
    bool rightEnableTmp;
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
            m_ooEnableLeftDrive();
            m_ooEnableRigthtDrive();
        }
    }
    else
    {
        if( enable.data )
        {
            m_ooDisableLeftDrive();
            m_ooDisableRightDrive();
        }
        else
        {
            LOG(Info) << "ooMotorPower : you are tryin to unpower the drive but they are already unpowered !" << endlog();
        }
    }

    //Attente de confirmation de l'action :
    whileTimeout(inLeftDriveEnable.readNewest(leftEnableTmp) != NoData && leftEnableTmp!=powerOn
            && inRightDriveEnable.readNewest(rightEnableTmp) != NoData && rightEnableTmp!=powerOn, timeout, 0.050);
    IfWhileTimeoutExpired(timeout)
    {
        LOG(Error) << "ooMotorPower : motor didn't switch power as required, timeout is over." << endlog();
        goto failed;
    }

    //remise ne mode vitesse des moteurs
    if( m_ooSetLeftOperationMode("speed")== false || m_ooSetRightOperationMode("speed") == false )
    {
        LOG(Error) << "ooMotorPower : could not switch back to speed mode." << endlog();
        goto failed;
    }

    LOG(Info) << "ooMotorPower : Motors power switched to " << powerOn << " properly." << endlog();
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool ProtokrotItf::srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = ooSetMotorPower(req.powerOn, req.timeout);
    return true;
}

bool ProtokrotItf::ooResetHml()
{
    bool res = true;
    res &= m_coResetIo();
    res &= m_coResetLeftDriving();
    res &= m_coResetRightDriving();
    return false;
}

bool ProtokrotItf::srvResetHml(ResetHml::Request& req, ResetHml::Response& res)
{
    res.success = ooResetHml();
    return true;
}

