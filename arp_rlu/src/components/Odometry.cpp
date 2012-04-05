/*
 * Odometry.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Odometry.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry )

Odometry::Odometry(const std::string& name):
        RluTaskContext(name)
{
    createOrocosInterface();
}






void Odometry::createOrocosInterface()
{
    addEventPort("inTime",inTime)
            .doc("");
    addPort("inParams",inParams)
            .doc("");

    addPort("inLeftDrivingSpeed",inLeftDrivingSpeed)
            .doc("");
    addPort("inRightDrivingSpeed",inRightDrivingSpeed)
            .doc("");
    addPort("inRearDrivingSpeed",inRearDrivingSpeed)
            .doc("");
    addPort("inLeftSteeringSpeed",inLeftSteeringSpeed)
            .doc("");
    addPort("inRightSteeringSpeed",inRightSteeringSpeed)
            .doc("");
    addPort("inRearSteeringSpeed",inRearSteeringSpeed)
            .doc("");

    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("");
    addPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("");
    addPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("");
    addPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("");
    addPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("");

    addPort("outTwist",outTwist)
            .doc("");
}
