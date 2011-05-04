/*
 * ProtokrotItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "ProtokrotItf.hpp"
#include "arp_hml_version.h";

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
    propLeftOdometerGain(TURN_TO_RAD),
    propRightOdometerGain(TURN_TO_RAD),
    propLeftSpeedGain(RAD_S_TO_RPM),
    propRightSpeedGain(RAD_S_TO_RPM),
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
            .doc("");
    addPort("outOdometryMeasures",outOdometryMeasures)
        .doc("");
    addPort("outIoStart",outIoStart)
        .doc("");
    addPort("outIoColorSwitch",outIoColorSwitch)
        .doc("");
    addPort("outEmergencyStop",outEmergencyStop)
        .doc("");

    /** Interface with INSIDE (hml !) **/
    addEventPort("inIoStart",inIoStart)
            .doc("");
    addPort("inIoColorSwitch",inIoColorSwitch)
            .doc("");//on n'est pas pressé pour connaitre la couleur
    addEventPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addEventPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");
    addPort("outLeftSpeedCmd",outLeftSpeedCmd)
            .doc("");
    addPort("outRightSpeedCmd",outRightSpeedCmd)
            .doc("");

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
	//publication des commandes de vitesse
	writeDifferentialCmd();

    //lecture des valeurs des odomètres
    readOdometers();

    //Lecture de la couleur
    readColorSwitch();

    //lecture du start
    readStart();
}

void ProtokrotItf::writeDifferentialCmd()
{
    DifferentialCommand cmd;
	struct timespec now;
	struct timespec delay;
	double delayInS;

    if(NewData==inDifferentialCmd.read(cmd))
    {
    	clock_gettime(CLOCK_MONOTONIC, &m_lastCmdTimestamp);
    	attrCurrentCmd = cmd;
        //ecriture des consignes moteurs
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
    if(NewData==inLeftDrivingPosition.read(odoValue))
    {
    	attrOdometers.odo_left = odoValue*propLeftOdometerGain;
    }
    if(NewData==inRightDrivingPosition.read(odoValue))
    {
    	attrOdometers.odo_right = odoValue*propRightOdometerGain;
    }
    outOdometryMeasures.write(attrOdometers);
}

void ProtokrotItf::readColorSwitch()
{
    bool io = false;
    StartColor colorSwitch;
    if(NewData==inIoColorSwitch.read(io))
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
	if(NewData==inIoStart.read(io))
	{
	    start.go = !io;
	    outIoStart.write(start);
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
