/*
 * ProtokrotItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "ProtokrotItf.hpp"
#include <ocl/Component.hpp>
#include <math/math.hpp>
#include "arp_hml_version.h";

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
    propRightSpeedGain(RAD_S_TO_RPM)
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
    if(NewData==inDifferentialCmd.read(cmd))
    {
        attrCurrentCmd = cmd;
        //ecriture des consignes moteurs
        outLeftSpeedCmd.write(attrCurrentCmd.v_left*propLeftSpeedGain);
        outRightSpeedCmd.write(attrCurrentCmd.v_right*propRightSpeedGain);
    }
    else
    {
        //TODO WLA on fait quoi dans ce cas ? rien ? on continue comme avant ?
        //attrCurrentCmd.v_left = 0;
        //attrCurrentCmd.v_right = 0;
    }
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

