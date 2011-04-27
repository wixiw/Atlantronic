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
    addPort("outOdometryMeasures",outOdometryMeasures)
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

    double dt = getPeriod();
    attrOdometers.odo_left += attrCurrentCmd.v_left*dt;
    attrOdometers.odo_right+= attrCurrentCmd.v_right*dt;

    outOdometryMeasures.write(attrOdometers);
}


//il ne faut inclure qu'une fois dans la librairie la macro ORO_CREATE_COMPONENT_TYPE()
ORO_CREATE_COMPONENT_LIBRARY()
