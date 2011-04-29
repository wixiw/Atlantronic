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

    if( res == true )
    {
        res = enableDrive();

    }

    return res;
}

void Faulhaber3268Bx4::updateHook()
{
    double speed = 0;

    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    EnterMutex();

    //mise à jour de la consigne de vitesse
    if( inSpeedCmd.readNewest(speed) == NewData )
    {
        *m_faulhaberCommand = F_CMD_V;
        *m_faulhaberCommandParameter = 7000*speed;
        outCommandedSpeed.write(7000*speed);
    }

    //lecture de la vitesse
    outMeasuredPosition.write( *m_measuredPosition );
    //lecture du courant
    outMeasuredCurrent.write( *m_measuredCurrent );

    //lecture de la dernière commande envoyée :
    outLastSentCommand.write( *m_faulhaberCommandReturn );
    outLastSentCommandParam.write( *m_faulhaberCommandReturnParameter );
    outLastSentCommandReturn.write( *m_faulhaberCommandReturnCode );

    attrState = ArdDs402::getStateFromCanStatusWord( *m_ds402State );

    LeaveMutex();
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
    *m_faulhaberCommandParameter = 7000*speed;
    LeaveMutex();
    outCommandedSpeed.write(7000*speed);
}

void Faulhaber3268Bx4::ooReadSpeed()
{
    int receivedData;
    CanDicoEntry speedMeasureSdo =  CanDicoEntry(propNodeId,0x6069, 0, 0, 0, 4);
    m_coReadInRemoteDico(speedMeasureSdo,&receivedData);
    cout << "speed = " << receivedData << endl;
}

void Faulhaber3268Bx4::ooEnableDrive(bool enable)
{
    if( enable )
    {
        enableDrive();
    }
    else
    {
        disableDrive();
    }
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
	EnterMutex();
    *m_faulhaberCommand = F_CMD_EN;
    *m_faulhaberCommandParameter = 0;
    LeaveMutex();
    //TODO WLA a remplacer par un check du resultat precedent
    usleep(1000*100);
    return true;
}

bool Faulhaber3268Bx4::disableDrive()
{
	EnterMutex();
    *m_faulhaberCommand = F_CMD_DI;
    *m_faulhaberCommandParameter = 0;
    LeaveMutex();
    //TODO WLA a remplacer par un check du resultat precedent
    usleep(1000*100);
    return true;
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
