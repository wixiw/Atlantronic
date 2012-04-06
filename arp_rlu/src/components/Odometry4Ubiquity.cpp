/*
 * Odometry4Ubiquity.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Odometry4Ubiquity.hpp"
#include <rtt/Component.hpp>

using namespace arp_math;
using namespace arp_model;
using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4Ubiquity )

Odometry4Ubiquity::Odometry4Ubiquity(const std::string& name):
RluTaskContext(name)
{
    createOrocosInterface();
}

void Odometry4Ubiquity::updateHook()
{
    ARDTaskContext::updateHook();

    double time;
    if( RTT::NewData != inTime.readNewest(time))
    {
        LOG( Warning ) << "No new data in inTime port : updateHook should not be externally trigger => return" << endlog();
        return;
    }

    UbiquityParams params;
    if( RTT::NoData == inParams.readNewest(params))
    {
        LOG( Warning ) << "No data in inParams port => return" << endlog();
        return;
    }

    double leftDrivingSpeed;
    if( RTT::NewData != inLeftDrivingSpeed.readNewest(leftDrivingSpeed))
    {
        LOG( Warning ) << "No new data in inLeftDrivingSpeed port => return" << endlog();
        return;
    }

    double rightDrivingSpeed;
    if( RTT::NewData != inRightDrivingSpeed.readNewest(rightDrivingSpeed))
    {
        LOG( Warning ) << "No new data in inRightDrivingSpeed port => return" << endlog();
        return;
    }

    double rearDrivingSpeed;
    if( RTT::NewData != inRearDrivingSpeed.readNewest(rearDrivingSpeed))
    {
        LOG( Warning ) << "No new data in inRearDrivingSpeed port => return" << endlog();
        return;
    }

//    double leftSteeringSpeed;
//    if( RTT::NewData != inLeftSteeringSpeed.readNewest(leftSteeringSpeed))
//    {
//        LOG( Warning ) << "No new data in inLeftSteeringSpeed port => return" << endlog();
//        return;
//    }
//
//    double rightSteeringSpeed;
//    if( RTT::NewData != inRightSteeringSpeed.readNewest(rightSteeringSpeed))
//    {
//        LOG( Warning ) << "No new data in inRightSteeringSpeed port => return" << endlog();
//        return;
//    }
//
//    double rearSteeringSpeed;
//    if( RTT::NewData != inRearSteeringSpeed.readNewest(rearSteeringSpeed))
//    {
//        LOG( Warning ) << "No new data in inRearSteeringSpeed port => return" << endlog();
//        return;
//    }

//    double leftDrivingPosition;
//    if( RTT::NewData != inLeftDrivingPosition.readNewest(leftDrivingPosition))
//    {
//        LOG( Warning ) << "No new data in inLeftDrivingPosition port => return" << endlog();
//        return;
//    }
//
//    double rightDrivingPosition;
//    if( RTT::NewData != inRightDrivingPosition.readNewest(rightDrivingPosition))
//    {
//        LOG( Warning ) << "No new data in inRightDrivingPosition port => return" << endlog();
//        return;
//    }
//
//    double rearDrivingPosition;
//    if( RTT::NewData != inRearDrivingPosition.readNewest(rearDrivingPosition))
//    {
//        LOG( Warning ) << "No new data in inRearDrivingPosition port => return" << endlog();
//        return;
//    }

    double leftSteeringPosition;
    if( RTT::NewData != inLeftSteeringPosition.readNewest(leftSteeringPosition))
    {
        LOG( Warning ) << "No new data in inLeftSteeringPosition port => return" << endlog();
        return;
    }

    double rightSteeringPosition;
    if( RTT::NewData != inRightSteeringPosition.readNewest(rightSteeringPosition))
    {
        LOG( Warning ) << "No new data in inRightSteeringPosition port => return" << endlog();
        return;
    }

    double rearSteeringPosition;
    if( RTT::NewData != inRearSteeringPosition.readNewest(rearSteeringPosition))
    {
        LOG( Warning ) << "No new data in inRearSteeringPosition port => return" << endlog();
        return;
    }

}



void Odometry4Ubiquity::createOrocosInterface()
{
    addEventPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inLeftDrivingSpeed",inLeftDrivingSpeed)
            .doc("Driving Speed in m/s for left turret");
    addPort("inRightDrivingSpeed",inRightDrivingSpeed)
            .doc("Driving Speed in m/s for right turret");
    addPort("inRearDrivingSpeed",inRearDrivingSpeed)
            .doc("Driving Speed in m/s for rear turret");
    addPort("inLeftSteeringSpeed",inLeftSteeringSpeed)
            .doc("Steering Speed in rad/s for left turret");
    addPort("inRightSteeringSpeed",inRightSteeringSpeed)
            .doc("Steering Speed in rad/s for right turret");
    addPort("inRearSteeringSpeed",inRearSteeringSpeed)
            .doc("Steering Speed in rad/s for rear turret");

    addPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("Driving Position in m for left turret");
    addPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("Driving Position in m for right turret");
    addPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("Driving Position in m for rear turret");
    addPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("Steering Position in rad for left turret");
    addPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("Steering Position in rad for right turret");
    addPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("Steering Position in rad for rear turret");

    addPort("outTwist",outTwist)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
}
