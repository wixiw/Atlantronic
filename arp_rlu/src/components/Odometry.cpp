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
    addPort("inLeftTurretSpeed",inLeftTurretSpeed)
            .doc("");
    addPort("inRightTurretSpeed",inRightTurretSpeed)
            .doc("");
    addPort("inRearTurretSpeed",inRearTurretSpeed)
            .doc("");
    addPort("inLeftTurretPosition",inLeftTurretPosition)
            .doc("");
    addPort("inRightTurretPosition",inRightTurretPosition)
            .doc("");
    addPort("inRearTurretPosition",inRearTurretPosition)
            .doc("");

    addPort("outTwist",outTwist)
            .doc("");
}
