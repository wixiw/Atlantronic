/*
 * ProtokrotItf.cpp
 *
 *  Created on: 26 oct. 2010
 *      Author: wla
 */

#include "ProtokrotItf.hpp"
#include <ocl/Component.hpp>

using namespace arp_hml;
using namespace arp_core;

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

    addPort("inDifferentialCmd",inDifferentialCmd)
            .doc("");
    addPort("inIoStart",inIoStart)
            .doc("");
    addPort("inIoColorSwitch",inIoColorSwitch)
            .doc("");
    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");

    addPort("outOdometryMeasures",outOdometryMeasures)
        .doc("");
    addPort("outIoStart",outIoStart)
        .doc("");
    addPort("outIoColorSwitch",outIoColorSwitch)
        .doc("");
    addPort("outEmergencyStop",outEmergencyStop)
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
    	attrOdometers.odo_left = odoValue;
    }
    if(NewData==inRightDrivingPosition.read(odoValue))
    {
    	attrOdometers.odo_right = odoValue;
    }
    outOdometryMeasures.write(attrOdometers);


}
