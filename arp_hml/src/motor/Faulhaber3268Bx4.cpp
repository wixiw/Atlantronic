/*
 * Faulhaber3268Bx4.cpp
 *
 *  Created on: 10 mars 2011
 *      Author: ard
 */

#include "Faulhaber3268Bx4.hpp"
#include <ocl/Component.hpp>
#include "can/dictionnary/CanARD.h"
#include "can/wrappers/CanARDDictionnaryAccessor.hpp"

using namespace arp_hml;
using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_hml::Faulhaber3268Bx4 )

Faulhaber3268Bx4::Faulhaber3268Bx4(const std::string& name) :
        CanOpenNode(name),
        ArdMotorItf(),
        attrState(ArdDs402::UnknownDs402State),
        propInvertDriveDirection(false),
        inSpeedCmd()
{
    outMeasuredPosition.write(0.0);
    outMeasuredCurrent.write(0.0);

    m_measuredPosition = CanARDDictionnaryAccessor::getINTEGER32Pointer(name,"MeasuredPosition");
    m_measuredCurrent = CanARDDictionnaryAccessor::getINTEGER16Pointer(name,"MeasuredCurrent");
    m_faulhaberCommand = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommand");
    m_faulhaberCommandParameter = CanARDDictionnaryAccessor::getUNS32Pointer(name,"FaulhaberCommandParameter");
    m_faulhaberCommandReturn = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommandReturn");
    m_faulhaberCommandReturnCode = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommandReturnCode");
    m_faulhaberCommandReturnParameter = CanARDDictionnaryAccessor::getUNS32Pointer(name,"FaulhaberCommandReturnParameter");
    m_ds402State = CanARDDictionnaryAccessor::getUNS16Pointer(name,"Ds402State");

    addAttribute("attrState",attrState);

    addProperty("propInvertDriveDirection",propInvertDriveDirection);

    addPort("inSpeedCmd",inSpeedCmd)
            .doc("");
    addPort("outCommandedSpeed",outCommandedSpeed)
        .doc("");
    addPort("outMeasuredPosition",outMeasuredPosition)
        .doc("");
    addPort("outMeasuredCurrent",outMeasuredCurrent)
        .doc("");
    addPort("outLastSentCommand",outLastSentCommand)
        .doc("");
    addPort("outLastSentCommandParam",outLastSentCommandParam)
        .doc("");
    addPort("outLastSentCommandReturn",outLastSentCommandReturn)
        .doc("");

    addOperation("ooSendSpeed", &Faulhaber3268Bx4::ooSendSpeed,this, OwnThread )
        .doc("")
        .arg("speed","");
    addOperation("ooReadSpeed", &Faulhaber3268Bx4::ooReadSpeed,this, OwnThread )
        .doc("");
    addOperation("ooEnableDrive", &Faulhaber3268Bx4::ooEnableDrive,this, OwnThread )
        .doc("");
    addOperation("ooDisableDrive", &Faulhaber3268Bx4::ooDisableDrive,this, OwnThread )
        .doc("");
    addOperation("ooFaulhaberCmd", &Faulhaber3268Bx4::ooFaulhaberCmd,this, OwnThread )
        .doc("");

}

bool Faulhaber3268Bx4::configureHook()
{
    bool res = true;

    if( m_measuredCurrent == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_measuredCurrent" << endlog();
    }
    if( m_measuredPosition == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_measuredPosition" << endlog();
    }
    if( m_faulhaberCommand == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_faulhaberCommand" << endlog();
    }
    if( m_faulhaberCommandParameter == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_faulhaberCommandParameter" << endlog();
    }
    if( m_faulhaberCommandReturn == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_faulhaberCommandReturn" << endlog();
    }
    if( m_faulhaberCommandReturnCode == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_faulhaberCommandReturnCode" << endlog();
    }
    if( m_faulhaberCommandReturnParameter == NULL )
    {
    	res = false;
    	LOG(Error) << "failed to configure : did not get CAN pointer m_faulhaberCommandReturnParameter" << endlog();
    }

    if( res == true )
    {
        res = CanOpenNode::configureHook();

    }

    return res;
}

void Faulhaber3268Bx4::updateHook()
{
    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    switch (m_mode)
    {
		case SPEED_CONTROL:
			speedMode();
			break;
		case OTHER:
			faulhaberCommandMode();
			break;

		default:
			LOG(Error) << "Unknown mode of operation" << endlog();
			break;
	}


    EnterMutex();
    //lecture de la vitesse
	if( propInvertDriveDirection )
	{
		outMeasuredPosition.write( - (*m_measuredPosition) );
	}
	else
	{
		outMeasuredPosition.write( *m_measuredPosition );
	}
    //lecture du courant
    outMeasuredCurrent.write( *m_measuredCurrent );
    //lecture de la dernière commande envoyée :
    outLastSentCommand.write( *m_faulhaberCommandReturn );
    outLastSentCommandParam.write( *m_faulhaberCommandReturnParameter );
    outLastSentCommandReturn.write( *m_faulhaberCommandReturnCode );
    //lecture de l'état DS402s
    attrState = ArdDs402::getStateFromCanStatusWord( *m_ds402State );
    LeaveMutex();
}

void Faulhaber3268Bx4::speedMode()
{
	int speed = 0;

    EnterMutex();
    //mise à jour de la consigne de vitesse
    if( inSpeedCmd.readNewest(speed) == NewData )
    {
    	if( propInvertDriveDirection )
    	{
    		speed = -speed;
    	}
        *m_faulhaberCommand = F_CMD_V;
        *m_faulhaberCommandParameter = (UNS32)(speed);
        outCommandedSpeed.write((int)(*m_faulhaberCommandParameter));
    }

    LeaveMutex();
}

void Faulhaber3268Bx4::faulhaberCommandMode()
{
	EnterMutex();
    *m_faulhaberCommand = m_faulhaberScriptCommand;
    *m_faulhaberCommandParameter = m_faulhaberScriptCommandParam;
    LeaveMutex();

    //temps que ma commande n'est pas prise en compte j'attend
    if( outLastSentCommand.getLastWrittenValue() != m_faulhaberScriptCommand )
    	return;

    if( outLastSentCommandReturn.getLastWrittenValue() != F_RET_OK )
    {
    	LOG(Error) << "faulhaber command " << m_faulhaberScriptCommand << " failed : return code " << outLastSentCommandReturn.getLastWrittenValue() << endlog();
    }
    else
    {
    	LOG(Info) << "faulhaber " << m_faulhaberScriptCommand << " command succeed" << endlog();
    }

    //retour en mode speed
    m_mode = SPEED_CONTROL;


}

void Faulhaber3268Bx4::stopHook()
{
    disableDrive();
    CanOpenNode::stopHook();
}

void Faulhaber3268Bx4::ooSendSpeed(int speed)
{
	EnterMutex();
    *m_faulhaberCommand = F_CMD_V;
    *m_faulhaberCommandParameter = speed;
    LeaveMutex();
    outCommandedSpeed.write(speed);
}

void Faulhaber3268Bx4::ooReadSpeed()
{
    int receivedData;
    CanDicoEntry speedMeasureSdo =  CanDicoEntry(propNodeId,0x6069, 0, 0, 0, 4);
    m_coReadInRemoteDico(speedMeasureSdo,&receivedData);
    cout << "speed = " << receivedData << endl;
}

void Faulhaber3268Bx4::ooEnableDrive()
{
	m_mode = OTHER;
	m_faulhaberScriptCommand = F_CMD_EN;
	m_faulhaberScriptCommandParam = 0;
}

void Faulhaber3268Bx4::ooDisableDrive()
{
	m_mode = OTHER;
	m_faulhaberScriptCommand = F_CMD_DI;
	m_faulhaberScriptCommandParam = 0;
}

void Faulhaber3268Bx4::ooFaulhaberCmd(int cmd, int param)
{
	m_mode = OTHER;
	m_faulhaberScriptCommand = (UNS8) cmd;
	m_faulhaberScriptCommandParam = (UNS32) param;
}

bool Faulhaber3268Bx4::setOperationMode(int operationMode)
{
    return false;
}

bool Faulhaber3268Bx4::init()
{
    return false;
}

bool Faulhaber3268Bx4::enableDrive()
{
	int chrono = 0;
	EnterMutex();
    *m_faulhaberCommand = F_CMD_EN;
    *m_faulhaberCommandParameter = 0;
    LeaveMutex();
    while( outLastSentCommand.getLastWrittenValue() != F_CMD_EN && chrono < propNmtTimeout )
    {
    	usleep(100*1000);
    	chrono+=100;
    }
    if( chrono >= propNmtTimeout )
    {
    	LOG(Error) << "timeout expired, failed to receive enableDrive return code" << endlog();
        return false;
    }


    if( outLastSentCommand.getLastWrittenValue() == F_CMD_EN
    		&& outLastSentCommandReturn.getLastWrittenValue() == F_RET_OK )
    	return true;
    else
    {
    	LOG(Error) << "failed to enable drive : return code " << outLastSentCommandReturn.getLastWrittenValue() << endlog();
        return false;
    }
}

bool Faulhaber3268Bx4::disableDrive()
{
	int chrono = 0;
	EnterMutex();
    *m_faulhaberCommand = F_CMD_DI;
    *m_faulhaberCommandParameter = 0;
    LeaveMutex();
    while( outLastSentCommand.getLastWrittenValue() != F_CMD_DI && chrono < propNmtTimeout )
    {
    	usleep(100*1000);
    	chrono+=100;
    }
    if( chrono >= propNmtTimeout )
    {
    	LOG(Error) << "timeout expired, failed to receive disableDrive return code" << endlog();
        return false;
    }


    if( outLastSentCommand.getLastWrittenValue() == F_CMD_DI
    		&& outLastSentCommandReturn.getLastWrittenValue() == F_RET_OK )
    	return true;
    else
    {
    	LOG(Error) << "failed to disable drive : return code " << outLastSentCommandReturn.getLastWrittenValue() << endlog();
        return false;
    }
}

bool Faulhaber3268Bx4::reset()
{
    return false;
}

bool Faulhaber3268Bx4::getLimitSwitchStatus()
{
    return false;
}

bool Faulhaber3268Bx4::startWatchdog()
{
    return false;
}

bool Faulhaber3268Bx4::stopWatchdog()
{
    return false;
}

bool Faulhaber3268Bx4::isInError()
{
    return false;
}

unsigned int Faulhaber3268Bx4::getError()
{
    return 0;
}
