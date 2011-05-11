/*
 * ArdMotor.cpp
 *
 *  Created on: 07 may 2011
 *      Author: wla
 */

#include "ArdMotorItf.hpp"

using namespace arp_hml;

bool ArdMotorItf::setOperationMode(operationMode_t operationMode)
{
	m_operationMode=operationMode;
	return true;
};

ArdMotorItf::operationMode_t ArdMotorItf::getOperationMode()
{
	return m_operationMode;
};

void ArdMotorItf::run()
{
	switch (getOperationMode())
	{
		case SPEED_CONTROL:
			runSpeed();
			break;
		case TORQUE_CONTROL:
			runTorque();
			break;
		case POSITION_CONTROL:
			runPosition();
			break;
		case HOMING:
			runHoming();
			break;
		case OTHER:
			runOther();
			break;
		default:
			break;
	}
}

double ArdMotorItf::getTorqueMeasure()
{
	return m_torqueMeasure;
};

void ArdMotorItf::setTorqueCmd(double torqueCmdNm )
{
	m_torqueCommand = torqueCmdNm;
};

double ArdMotorItf::getSpeedMeasure()
{
	return m_speedMeasure;
};

void ArdMotorItf::setSpeedCmd(double speedCmdRadS )
{
	m_speedCommand = speedCmdRadS;
};

double ArdMotorItf::getPositionMeasure()
{
	return m_positionMeasure;
};

void ArdMotorItf::setPositionCmd(double positionCmdRadS )
{
	m_positionCommand=positionCmdRadS;
};

std::string ArdMotorItf::getStringFromMode(operationMode_t mode)
{
	switch (mode)
	{
		case SPEED_CONTROL:
			return "speed";
		case TORQUE_CONTROL:
			return "torque";
		case POSITION_CONTROL:
			return "position";
		case HOMING:
			return "homing";
		case OTHER:
			return "other";
		default:
			return "unknown";
	}
}

ArdMotorItf::operationMode_t ArdMotorItf::getModeFromString( std::string mode )
{
	if( mode == "speed" )
		return SPEED_CONTROL;
	else if( mode == "torque" )
		return TORQUE_CONTROL;
	else if( mode == "position" )
		return POSITION_CONTROL;
	else if( mode == "homing" )
		return HOMING;
	else if( mode == "other" )
		return OTHER;
	else
		return OTHER;
}

