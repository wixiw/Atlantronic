/*
 * ProtokrotItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "ProtokrotItf.hpp"
#include <ocl/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std_msgs;

ORO_LIST_COMPONENT_TYPE( arp_hml::ProtokrotItf )

ProtokrotItf::ProtokrotItf(const std::string& name):
    ARDTaskContext(name),
    attrCurrentCmd(),
    attrOdometers()
{
    //TODO WLA : workaround en attendant de trouver dans quel dossier on est lancé dans ROS
    attrPropertyPath = "/opt/ros/ard/arp_hml/script/orocos/conf";
    attrScriptPath = "/opt/ros/ard/arp_hml/script/orocos/ops";
    attrStateMachinePath = "/opt/ros/ard/arp_hml/script/orocos/osd";

    addAttribute("attrCurrentCmd", attrCurrentCmd);
    addAttribute("attrOdometers", attrOdometers);

    /** Interface with OUTSIDE (master, ODS, RLU) **/
    addPort("inDifferentialCmd",inDifferentialCmd)
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
    addPort("inIoStart",inIoStart)
            .doc("");
    addPort("inIoColorSwitch",inIoColorSwitch)
            .doc("");
    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");
    addPort("outLeftSpeedCmd",outLeftSpeedCmd)
            .doc("");
    addPort("outRightSpeedCmd",outRightSpeedCmd)
            .doc("");
}

void ProtokrotItf::updateHook()
{
    DifferentialCommand cmd;
    if(NewData==inDifferentialCmd.read(cmd))
    {
        attrCurrentCmd = cmd;
    }
    else
    {
        //on fait quoi dans ce cas ? rien ? on continue comme avant ?
        //attrCurrentCmd.v_left = 0;
        //attrCurrentCmd.v_right = 0;
    }

    //lecture des valeurs des odomètres
    double odoValue;
    if(NewData==inLeftDrivingPosition.read(odoValue))
    {
    	attrOdometers.odo_left = odoValue*2*PI/24000/1.785;
    }
    if(NewData==inRightDrivingPosition.read(odoValue))
    {
    	attrOdometers.odo_right = odoValue*2*PI/24000/1.785;
    }
    outOdometryMeasures.write(attrOdometers);

    //Lecture de la couleur
    bool io = false;
    StartColor colorSwitch;
    inIoColorSwitch.readNewest(io);
    if(io)
    	colorSwitch.color = "red";
    else
    	colorSwitch.color = "blue";
    outIoColorSwitch.write(colorSwitch);
    //lecture du start
    Start start;
    io = false;
    inIoStart.readNewest(io);
    start.go = !io;
    outIoStart.write(start);

    //ecriture des consignes moteurs
    outLeftSpeedCmd.write(attrCurrentCmd.v_left*30/PI*14);
    outRightSpeedCmd.write(attrCurrentCmd.v_right*30/PI*14);


}
