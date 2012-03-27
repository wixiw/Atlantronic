/*
 * RosHmlItf.cpp
 *
 *  Created on: 03 Fev 2012
 *      Author: wla
 */

#include "RosHmlItf.hpp"

#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std_msgs;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::RosHmlItf )

RosHmlItf::RosHmlItf(const std::string& name):
	HmlTaskContext(name),
    attrCurrentCmd(),
    attrOdometers(),
    propSpeedCmdMaxDelay(1.000),
    m_receivedPartialPosition(false)
{
    createOrocosInterface();
    createRosInterface();
}

bool RosHmlItf::configureHook()
{
    bool res = true;
    HmlTaskContext::configureHook();

    res &= getOperation("HmlMonitor" ,  "coSetMotorPower",          m_coSetMotorPower);
    res &= getOperation("HmlMonitor",   "coSetDrivingMotorPower",   m_coSetDrivingMotorPower);
    res &= getOperation("HmlMonitor",   "coSetSteeringMotorPower",  m_coSetSteeringMotorPower);

    res &= getOperation("HmlMonitor"    , "ooResetHml",  m_ooResetHml);

    if( res == false )
    {
        LOG(Error) << "failed to configure : did not get operations" << endlog();
    }

    return res;
}

void RosHmlItf::updateHook()
{
	HmlTaskContext::updateHook();

	//publication des commandes de vitesse
	//writeOmniCmd();

    //lecture des valeurs des odomètres
    readOdometers();

    //lecture du start
    readStart();

    //lecture de enable
    readDriveEnable();

    //lecture des vitesses
    readSpeed();

    //lecture du blocage roues
    readWheelBlocked();
}

/*
void RosHmlItf::writeOmniCmd()
{
    OmniCommand cmd;
	struct timespec now;
	double delayInS;

    if(NewData==inOmniCmd.read(cmd))
    {
    	//ecriture des consignes moteurs
        Bool enable = outDriveEnable.getLastWrittenValue();
    	if( enable.data )
    	{
			clock_gettime(CLOCK_MONOTONIC, &m_lastCmdTimestamp);
			attrCurrentCmd = cmd;
    	}
    	//si les moteurs sont disable on n'envoit pas de consigne
    	else
    	{
            attrCurrentCmd.v_left_driving = 0;
            attrCurrentCmd.v_right_driving = 0;
            attrCurrentCmd.v_rear_driving = 0;
            attrCurrentCmd.p_left_steering = 0;
            attrCurrentCmd.p_right_steering = 0;
            attrCurrentCmd.p_rear_steering = 0;
    	}
    }
    else
    {
    	//si on n'a pas reçu de commande, au bout d'une seconde on "met les freins"
    	clock_gettime(CLOCK_MONOTONIC, &now);
    	delayInS = delta_t(m_lastCmdTimestamp, now);
    	if( delayInS >= propSpeedCmdMaxDelay )
        {
            attrCurrentCmd.v_left_driving = 0;
            attrCurrentCmd.v_right_driving = 0;
            attrCurrentCmd.v_rear_driving = 0;
            attrCurrentCmd.p_left_steering = 0;
            attrCurrentCmd.p_right_steering = 0;
            attrCurrentCmd.p_rear_steering = 0;
        }
    }

    outLeftDrivingSpeedCmd.write(attrCurrentCmd.v_left_driving);
    outRightDrivingSpeedCmd.write(attrCurrentCmd.v_right_driving);
    outRearDrivingSpeedCmd.write(attrCurrentCmd.v_rear_driving);
    outLeftSteeringPositionCmd.write(attrCurrentCmd.p_left_steering);
    outRightSteeringPositionCmd.write(attrCurrentCmd.p_right_steering);
    outRearSteeringPositionCmd.write(attrCurrentCmd.p_rear_steering);
}
*/

void RosHmlItf::readOdometers()
{
    double odoValueLeftDriving;
    double odoValueRightDriving;
    double odoValueRearDriving;
    double odoValueLeftSteering;
    double odoValueRightSteering;
    double odoValueRearSteering;

    double odoTimeLeftDriving;
    double odoTimeRightDriving;
    double odoTimeRearDriving;
    double odoTimeLeftSteering;
    double odoTimeRightSteering;
    double odoTimeRearSteering;

    if(         NoData != inLeftDrivingPositionTime.readNewest(odoTimeLeftDriving)
            &&  NoData != inRightDrivingPositionTime.readNewest(odoTimeRightDriving)
            &&  NoData != inRearDrivingPositionTime.readNewest(odoTimeRearDriving)
            &&  NoData != inLeftSteeringPositionTime.readNewest(odoTimeLeftSteering)
            &&  NoData != inRightSteeringPositionTime.readNewest(odoTimeRightSteering)
            &&  NoData != inRearSteeringPositionTime.readNewest(odoTimeRearSteering)
    )
    {
    	if(         odoTimeLeftDriving != odoTimeRightDriving
    	        ||  odoTimeRightDriving != odoTimeRearDriving
    	        ||  odoTimeRearDriving != odoTimeLeftSteering
    	        ||  odoTimeLeftSteering != odoTimeRightSteering
                ||  odoTimeRightSteering != odoTimeRearSteering
    	    )
    	{
    	    if( m_receivedPartialPosition == true )
    	    {
    	        LOG(Error) << "Did not receive the two Odometry measures in time-> Reseting HML" << endlog();
    	        cerr << "Did not receive the two Odometry measures in time-> Reseting HML" << endl;
    	        //ooResetHml();
    	        m_receivedPartialPosition = false;
    	        //ooSetMotorPower(true,1.0);
    	    }
    	    else
    	    {
    	        m_receivedPartialPosition = true;
    	    }
    	}
    	else
    	{
    	    //on ne reproduit pas le message si c'est le dernier qui a été publié
    	    if( attrOdometers.time != odoTimeLeftDriving )
    	    {
                m_receivedPartialPosition = false;
                inLeftDrivingPosition.readNewest(odoValueLeftDriving);
                inRightDrivingPosition.readNewest(odoValueRightDriving);
                inRearDrivingPosition.readNewest(odoValueRearDriving);
                inLeftSteeringPosition.readNewest(odoValueLeftSteering);
                inRightSteeringPosition.readNewest(odoValueRightSteering);
                inRearSteeringPosition.readNewest(odoValueRearSteering);
                attrOdometers.odo_left_driving = odoValueLeftDriving;
                attrOdometers.odo_right_driving = odoValueRightDriving;
                attrOdometers.odo_rear_driving = odoValueRearDriving;
                attrOdometers.odo_left_steering = odoValueLeftSteering;
                attrOdometers.odo_right_steering = odoValueRightSteering;
                attrOdometers.odo_rear_steering = odoValueRearSteering;
                attrOdometers.time = odoTimeLeftDriving;
                outOdometryMeasures.write(attrOdometers);
    	    }
    	}
    }
}

void RosHmlItf::readStart()
{
	bool io = false;
	Start start;
	if(NewData==inIoStart.readNewest(io))
	{
	    start.go = !io;
	    outIoStart.write(start);
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

void RosHmlItf::readSpeed()
{
	double leftDrivingSpeed = 0.0;
	double rightDrivingSpeed = 0.0;
	double rearDrivingSpeed = 0.0;

    double leftSteeringPosition = 0.0;
    double rightSteeringPosition = 0.0;
    double rearSteeringPosition = 0.0;

	OmniCommand speedMeasure;
	if( NoData != inLeftDrivingSpeedMeasure.readNewest(leftDrivingSpeed)
	 && NoData != inRightDrivingSpeedMeasure.readNewest(rightDrivingSpeed)
	 && NoData != inRearDrivingSpeedMeasure.readNewest(rearDrivingSpeed)

	 && NoData != inLeftSteeringPosition.readNewest(leftSteeringPosition)
     && NoData != inRightSteeringPosition.readNewest(rightSteeringPosition)
     && NoData != inRearSteeringPosition.readNewest(rearSteeringPosition)
	)
	{
		speedMeasure.v_left_driving = leftDrivingSpeed;
		speedMeasure.v_right_driving = rightDrivingSpeed;
		speedMeasure.v_rear_driving = rearDrivingSpeed;

        speedMeasure.p_left_steering = leftSteeringPosition;
        speedMeasure.p_right_steering = rightSteeringPosition;
        speedMeasure.p_rear_steering = rearSteeringPosition;
		outOmniSpeedMeasure.write(speedMeasure);
	}
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


//--------------------------------------------------------------------------------------------------


bool RosHmlItf::srvSetMotorPower(SetMotorPower::Request& req, SetMotorPower::Response& res)
{
    res.success = m_coSetMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvSetDrivingMotorPower(SetDrivingMotorPower::Request& req, SetDrivingMotorPower::Response& res)
{
    res.success = m_coSetSteeringMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvSetSteeringMotorPower(SetSteeringMotorPower::Request& req, SetSteeringMotorPower::Response& res)
{
    res.success = m_coSetDrivingMotorPower(req.powerOn);
    return res.success;
}

bool RosHmlItf::srvGetVersion(GetVersion::Request& req, GetVersion::Response& res)
{
    res.version = m_coGetVersion();
    return true;
}

bool RosHmlItf::srvResetHml(ResetHml::Request& req, ResetHml::Response& res)
{
    res.success = m_ooResetHml();
    return res.success;
}


//-------------------------------------------------------------------------------------------------
void RosHmlItf::createOrocosInterface()
{
    addAttribute("attrCurrentCmd", attrCurrentCmd);
    addAttribute("attrOdometers", attrOdometers);

    addProperty("propSpeedCmdMaxDelay", propSpeedCmdMaxDelay)
        .doc("Maximal delay beetween 2 received Differential commands. If this delay is overrun, a speed of 0 is sent on each motor. In s ");

    /** Interface with OUTSIDE (master, ODS, RLU) **/
    addEventPort("inOmniCmd",inOmniCmd)
            .doc("Speed command for the 6 motors motor");
    addPort("outOdometryMeasures",outOdometryMeasures)
        .doc("Odometers value from the motors assembled in an 'Odo' Ros message");
    addPort("outOmniSpeedMeasure",outOmniSpeedMeasure)
        .doc("Speed measures for left and right motor");
    addPort("outIoStart",outIoStart)
        .doc("Value of the start. GO is true when it is not in, go is false when the start is in");
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

    /** Interface with INSIDE (hml !) **/
    addEventPort("inIoStart",inIoStart)
            .doc("HW value of the start switch. It is true when the start is in");

    addEventPort("inLeftDrivingPosition",inLeftDrivingPosition)
            .doc("Value of the left driving odometer in rad on the wheel axe");
    addPort("inLeftDrivingPositionTime",inLeftDrivingPositionTime)
            .doc("");
    addEventPort("inRightDrivingPosition",inRightDrivingPosition)
            .doc("Value of the right driving odometer in rad on the wheel axe");
    addPort("inRightDrivingPositionTime",inRightDrivingPositionTime)
            .doc("");
    addEventPort("inRearDrivingPosition",inRearDrivingPosition)
            .doc("Value of the rear driving odometer in rad on the wheel axe");
    addPort("inRearDrivingPositionTime",inRearDrivingPositionTime)
            .doc("");
    addEventPort("inLeftSteeringPosition",inLeftSteeringPosition)
            .doc("Value of the left steering odometer in rad on the wheel axe");
    addPort("inLeftSteeringPositionTime",inLeftSteeringPositionTime)
            .doc("");
    addEventPort("inRightSteeringPosition",inRightSteeringPosition)
            .doc("Value of the right steering odometer in rad on the wheel axe");
    addPort("inRightSteeringPositionTime",inRightSteeringPositionTime)
            .doc("");
    addEventPort("inRearSteeringPosition",inRearSteeringPosition)
            .doc("Value of the rear steering odometer in rad on the wheel axe");
    addPort("inRearSteeringPositionTime",inRearSteeringPositionTime)
            .doc("");

    addPort("inLeftDrivingSpeedMeasure",inLeftDrivingSpeedMeasure)
            .doc("Value of the left driving speed in rad/s on the wheel axe");
    addPort("inRightDrivingSpeedMeasure",inRightDrivingSpeedMeasure)
            .doc("Value of the right driving speed in rad/s on the wheel axe");
    addPort("inRearDrivingSpeedMeasure",inRearDrivingSpeedMeasure)
            .doc("Value of the rear driving speed in rad/s on the wheel axe");
    addPort("inLeftSteeringSpeedMeasure",inLeftSteeringSpeedMeasure)
            .doc("Value of the left steering speed in rad/s on the wheel axe");
    addPort("inRightSteeringSpeedMeasure",inRightSteeringSpeedMeasure)
            .doc("Value of the right steering speed in rad/s on the wheel axe");
    addPort("inRearSteeringSpeedMeasure",inRearSteeringSpeedMeasure)
            .doc("Value of the rear steering speed in rad/s on the wheel axe");

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
    m_srvSetMotorPower = nh.advertiseService("/Protokrot/setMotorPower", &RosHmlItf::srvSetMotorPower, this);
    m_srvSetMotorPower = nh.advertiseService("/Protokrot/setDrivingMotorPower", &RosHmlItf::srvSetDrivingMotorPower, this);
    m_srvSetMotorPower = nh.advertiseService("/Protokrot/setSteeringMotorPower", &RosHmlItf::srvSetSteeringMotorPower, this);
    m_srvResetHml = nh.advertiseService("/Protokrot/resetHml", &RosHmlItf::srvResetHml, this);
}

