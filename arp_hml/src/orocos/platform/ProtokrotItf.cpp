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
    propSpeedCmdMaxDelay(1.000)
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
    addEventPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("Value of the right odometer in rad on the wheel axe");
    addPort("inLeftSpeedMeasure",inLeftSpeedMeasure)
            .doc("Value of the left speed in rad/s on the wheel axe");
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
}

string ProtokrotItf::coGetCoreVersion()
{
	return "not implemented yet";
}

string ProtokrotItf::coGetHmlVersion()
{
	return ARP_HML_VERSION;
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

    //lecture des vitesses
    readSpeed();
}

void ProtokrotItf::writeDifferentialCmd()
{
    DifferentialCommand cmd;
	struct timespec now;
	struct timespec delay;
	double delayInS;

    if(NewData==inDifferentialCmd.read(cmd))
    {
    	//ecriture des consignes moteurs
    	if( outDriveEnable.getLastWrittenValue() )
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
    	delta_t(&delay, &m_lastCmdTimestamp, &now);
    	delayInS = delay.tv_sec+(double)(delay.tv_nsec)/1E9;
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
    double odoValue;
    if(NewData==inLeftDrivingPosition.readNewest(odoValue))
    {
    	attrOdometers.odo_left = odoValue*propLeftOdometerGain;
    }
    if(NewData==inRightDrivingPosition.readNewest(odoValue))
    {
    	attrOdometers.odo_right = odoValue*propRightOdometerGain;
    }
    outOdometryMeasures.write(attrOdometers);
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

	if( leftDriveEnable && rightDriveEnable )
		outDriveEnable.write( true );
	else
		outDriveEnable.write( false );
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

void ProtokrotItf::delta_t(struct timespec *interval, struct timespec *begin, struct timespec *now)
{
	interval->tv_nsec = now->tv_nsec - begin->tv_nsec; /* Subtract 'decimal fraction' first */
	if(interval->tv_nsec < 0 )
	{
		interval->tv_nsec += 1000000000; /* Borrow 1sec from 'tv_sec' if subtraction -ve */
		interval->tv_sec = now->tv_sec - begin->tv_sec - 1; /* Subtract whole number of seconds and return 1 */
	}
	else
	{
		interval->tv_sec = now->tv_sec - begin->tv_sec; /* Subtract whole number of seconds and return 0 */
	}
}
