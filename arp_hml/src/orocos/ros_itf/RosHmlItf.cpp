/*
 * RosHmlItf.cpp
 *
 *  Created on: 03 Fev 2012
 *      Author: wla
 */

#include "RosHmlItf.hpp"

#include <rtt/Component.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std_msgs;
using namespace std;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::RosHmlItf )

RosHmlItf::RosHmlItf(const std::string& name):
	HmlTaskContext(name)
{
    createOrocosInterface();
    createRosInterface();
}

bool RosHmlItf::configureHook()
{
    bool res = true;
    HmlTaskContext::configureHook();

    res &= getOperation("HmlMonitor",   "coSetMotorPower",          m_coSetMotorPower);
    res &= getOperation("HmlMonitor",   "coSetDrivingMotorPower",   m_coSetDrivingMotorPower);
    res &= getOperation("HmlMonitor",   "coSetSteeringMotorPower",  m_coSetSteeringMotorPower);
    getOperation("HmlMonitor",   "ooSetDrivingOperationMode",  m_ooSetDrivingOperationMode );
    getOperation("HmlMonitor",   "ooSetSteeringOperationMode",  m_ooSetSteeringOperationMode );
    res &= getOperation("HmlMonitor",   "ooResetHml",               m_ooResetHml);
    //don't care if those are missing
    getOperation("HmlMonitor",          "coGetHmlVersion",             m_coGetVersion);

    if( getPeer("UbiquitySimul") != NULL )
        res &=getOperation("UbiquitySimul", "ooSetRealSimulPosition",  m_ooSetPosition);



    if( res == false )
    {
        LOG(Error) << "failed to configure : did not get operations" << endlog();
    }

    return res;
}

void RosHmlItf::updateHook()
{
	HmlTaskContext::updateHook();

    //lecture des Io du woodhead
    readIo();

    //lecture de enable
    readDriveEnable();

    //lecture du blocage roues
    readWheelBlocked();

    //lecture de la demande de blocage robot
    readBlockRobot();

    bool pouette;
    Bool homingDone;
    if( NoData != inIsHomingDone.readNewest(pouette) )
    {
        homingDone.data = pouette;
        outIsHomingDone.write(homingDone);
    }



    Pose poseOut;
    Pose2D poseIn;
    if( NoData != inRealPosition.readNewest(poseIn) )
    {
        poseOut.x = poseIn.x();
        poseOut.y = poseIn.y();
        poseOut.theta = poseIn.h();
        outRealPosition.write(poseOut);
    }

}

void RosHmlItf::readIo()
{
	bool io = false;
	Start start;
	if(NewData==inIoStart.readNewest(io))
	{
	    start.go = !io;
	    outIoStart.write(start);
	}

    StartColor startColor;
    if(NewData==inIoStartColor.readNewest(io))
    {
        if( io )
            startColor.color = "red";
        else
            startColor.color = "purple";
        outIoStartColor.write(startColor);
    }

    Obstacle frontLeftObstacle;
    if(NewData==inIoFrontLeftObstacle.readNewest(io))
    {
        frontLeftObstacle.detected = io;
        outFrontLeftObstacle.write(frontLeftObstacle);
    }

    Obstacle frontRightObstacle;
    if(NewData==inIoFrontRightObstacle.readNewest(io))
    {
        frontRightObstacle.detected = io;
        outFrontRightObstacle.write(frontRightObstacle);
    }

    Obstacle rearObstacle;
    if(NewData==inIoRearObstacle.readNewest(io))
    {
        rearObstacle.detected = io;
        outRearObstacle.write(rearObstacle);
    }
}

void RosHmlItf::readDriveEnable()
{
	bool steeringMotorsEnable = false;
	bool drivingMotorsEnable = false;
	bool motorsEnable = false;

	inDrivingMotorsEnable.readNewest(steeringMotorsEnable);
	inSteeringMotorsEnable.readNewest(drivingMotorsEnable);
	inMotorsEnable.readNewest(motorsEnable);

	Bool enable;
	enable.data = steeringMotorsEnable;
	outDrivingMotorsEnable.write( enable );
    enable.data = drivingMotorsEnable;
    outSteeringMotorsEnable.write( enable );
    enable.data = motorsEnable;
    outMotorsEnable.write( enable );
}

void RosHmlItf::readWheelBlocked()
{
    bool leftWheelBlocked = false;
    bool rightWheelBlocked = false;
    bool rearWheelBlocked = false;

    Bool blocked;
    inLeftDrivingBlocked.readNewest(leftWheelBlocked);
    inRightDrivingBlocked.readNewest(rightWheelBlocked);
    inRearDrivingBlocked.readNewest(rearWheelBlocked);
    blocked.data = leftWheelBlocked || rightWheelBlocked || rearWheelBlocked;
    outWheelBlocked.write(blocked);
}

void RosHmlItf::readBlockRobot()
{
    Bool blockRobot;
    inBlockRobot.readNewest(blockRobot);
    outBlockRobot.write(blockRobot.data);
}

//--------------------------------------------------------------------------------------------------


bool RosHmlItf::srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = m_coSetMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvSetDrivingMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = m_coSetDrivingMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvSetSteeringMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = m_coSetSteeringMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvSetDrivingOperationMode(SetMotorMode::Request& req, SetMotorMode::Response& res)
{
    res.success = m_ooSetDrivingOperationMode(req.mode);
    return res.success;
}

bool RosHmlItf::srvSetSteeringOperationMode(SetMotorMode::Request& req, SetMotorMode::Response& res)
{
    res.success = m_ooSetSteeringOperationMode(req.mode);
    return res.success;
}

bool RosHmlItf::srvGetVersion(GetVersion::Request& req, GetVersion::Response& res)
{
    res.version = m_coGetVersion();
    return true;
}

bool RosHmlItf::srvSetRealSimulPosition(SetPosition::Request& req, SetPosition::Response& res)
{
    if( getPeer("UbiquitySimul") != NULL )
    {
        Pose2D p(req.x,req.y,req.theta);
        res.success = m_ooSetPosition(p);
    }
    else
    {
        LOG(Error) << "You should not call srvSetRealSimulPosition outside simulation" << endlog();
        res.success = false;
    }
    return res.success;
}

bool RosHmlItf::srvResetHml(ResetHml::Request& req, ResetHml::Response& res)
{
    res.success = m_ooResetHml();
    return res.success;
}


//-------------------------------------------------------------------------------------------------
void RosHmlItf::createOrocosInterface()
{
    /** Interface with OUTSIDE (master, ODS, RLU) **/
    addPort("outIoStart",outIoStart)
        .doc("Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoStartColor",outIoStartColor)
        .doc("Value of the color switch");
    addPort("outFrontLeftObstacle",outFrontLeftObstacle)
        .doc("Value of the front left obstacle detector");
    addPort("outFrontRightObstacle",outFrontRightObstacle)
        .doc("Value of the front right obstacle detector");
    addPort("outRearObstacle",outRearObstacle)
        .doc("Value of the rear obstacle detector");

    addPort("outEmergencyStop",outEmergencyStop)
        .doc("Is true when HML thinks the emergency stop button is active");
    addPort("outDrivingMotorsEnable",outDrivingMotorsEnable)
        .doc("Is true when the 3 driving motors are enabled. Since this port is false, drive speed are forced to 0");
    addPort("outSteeringMotorsEnable",outSteeringMotorsEnable)
        .doc("Is true when the 3 steering motors are enabled. Since this port is false, drive speed are forced to 0");
    addPort("outMotorsEnable",outMotorsEnable)
        .doc("Is true when the 6 motors are enabled. Since this port is false, drive speed are forced to 0");
    addPort("outWheelBlocked",outWheelBlocked)
         .doc("Is true when one of the 3 driving wheel is blocked");
    addPort("outRealPosition",outRealPosition)
        .doc("Position calculated by the simulation (is just a type conversion from same named input port)");
    addPort("outIsHomingDone",outIsHomingDone)
        .doc("Is true when the 3 steering motors have finished their homing command");
    addPort("inBlockRobot",inBlockRobot)
            .doc("Is true when in simulation someone is asking to arbitrary block the wheels");

    /** Interface with INSIDE (hml !) **/
    addPort("inIoStart",inIoStart)
            .doc("HW value of the start switch. It is true when the start is in");
    addPort("inIoStartColor",inIoStartColor)
            .doc("");
    addPort("inIoFrontLeftObstacle",inIoFrontLeftObstacle)
            .doc("");
    addPort("inIoFrontRightObstacle",inIoFrontRightObstacle)
            .doc("");
    addPort("inIoRearObstacle",inIoRearObstacle)
            .doc("");
    addPort("inRealPosition",inRealPosition)
            .doc("Position calculated by simulation");
    addPort("inMotorMeasures",inMotorMeasures)
            .doc("");
    addPort("inIsHomingDone",inIsHomingDone)
            .doc("");



    addPort("inLeftDrivingBlocked",inLeftDrivingBlocked)
            .doc("Left driving motor is blocked");
    addPort("inRightDrivingBlocked",inRightDrivingBlocked)
            .doc("Right driving motor is blocked");
    addPort("inRearDrivingBlocked",inRearDrivingBlocked)
            .doc("Rear driving motor is blocked");
    addPort("inLeftSteeringBlocked",inLeftSteeringBlocked)
            .doc("Left steering motor is blocked");
    addPort("inRightSteeringBlocked",inRightSteeringBlocked)
            .doc("Right steering motor is blocked");
    addPort("inRearSteeringBlocked",inRearSteeringBlocked)
            .doc("Rear steering motor is blocked");

    addPort("outBlockRobot",outBlockRobot)
                .doc("Is true when in simulation someone is asking to arbitrary block the wheels");

    addPort("outLeftDrivingSpeedCmd",outLeftDrivingSpeedCmd)
            .doc("Speed command for the left driving motor in rad/s on the reductor output");
    addPort("outRightDrivingSpeedCmd",outRightDrivingSpeedCmd)
            .doc("Speed command for the right driving motor in rad/s on the reductor output");
    addPort("outRearDrivingSpeedCmd",outRearDrivingSpeedCmd)
            .doc("Speed command for the rear driving motor in rad/s on the reductor output");

    addPort("outLeftSteeringPositionCmd",outLeftSteeringPositionCmd)
            .doc("Position command for the left steering motor in rad/s on the reductor output");
    addPort("outRightSteeringPositionCmd",outRightSteeringPositionCmd)
            .doc("Position command for the right steering motor in rad/s on the reductor output");
    addPort("outRearSteeringPositionCmd",outRearSteeringPositionCmd)
            .doc("Position command for the rear steering motor in rad/s on the reductor output");

    addPort("outLeftDrivingTorqueCmd",outLeftDrivingTorqueCmd)
        .doc("Torque command for the left driving motor in Nm on the reductor output");
    addPort("outRightDrivingTorqueCmd",outRightDrivingTorqueCmd)
        .doc("Torque command for the right driving motor in Nm on the reductor output");
    addPort("outRearDrivingTorqueCmd",outRearDrivingTorqueCmd)
        .doc("Torque command for the rear driving motor in Nm on the reductor output");
    addPort("outLeftSteeringTorqueCmd",outLeftSteeringTorqueCmd)
        .doc("Torque command for the left steering motor in Nm on the reductor output");
    addPort("outRightSteeringTorqueCmd",outRightSteeringTorqueCmd)
        .doc("Torque command for the right steering motor in Nm on the reductor output");
    addPort("outRearSteeringTorqueCmd",outRearSteeringTorqueCmd)
        .doc("Torque command for the rear steering motor in Nm on the reductor output");
}

void RosHmlItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvSetMotorPower = nh.advertiseService("/Ubiquity/setMotorPower", &RosHmlItf::srvSetMotorPower, this);
    m_srvSetDrivingMotorPower = nh.advertiseService("/Ubiquity/setDrivingMotorPower", &RosHmlItf::srvSetDrivingMotorPower, this);
    m_srvSetSteeringMotorPower = nh.advertiseService("/Ubiquity/setSteeringMotorPower", &RosHmlItf::srvSetSteeringMotorPower, this);
    m_srvSetDrivingOperationMode = nh.advertiseService("/Ubiquity/setDrivingOperationMode", &RosHmlItf::srvSetDrivingOperationMode, this);
    m_srvSetDrivingOperationMode = nh.advertiseService("/Ubiquity/setSteeringOperationMode", &RosHmlItf::srvSetSteeringOperationMode, this);
    m_srvSetRealSimulPosition = nh.advertiseService("/Ubiquity/setRealSimulPosition", &RosHmlItf::srvSetRealSimulPosition, this);
    m_srvResetHml = nh.advertiseService("/Ubiquity/resetHml", &RosHmlItf::srvResetHml, this);
    m_srvGetVersion = nh.advertiseService("/Ubiquity/getHmlVersion", &RosHmlItf::srvGetVersion, this);
}

