/*
 * Faulhaber3268Bx4.cpp
 *
 *  Created on: 10 mars 2011
 *      Author: ard
 */

#include "Faulhaber3268Bx4.hpp"
#include <ocl/Component.hpp>
#include "can/dictionnary/CanARD.h"

using namespace arp_hml;
using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_hml::Faulhaber3268Bx4 )

Faulhaber3268Bx4::Faulhaber3268Bx4(const std::string& name) :
        CanOpenNode(name),
        ArdMotorItf(),
        attrState(ArdDs402::UnknownDs402State),
        inSpeedCmd()
{
    outMeasuredSpeed.write(0.0);

    addAttribute("attrState",attrState);

    addPort("inSpeedCmd",inSpeedCmd)
            .doc("");
    addPort("outCommandedSpeed",outCommandedSpeed)
        .doc("");
    addPort("outMeasuredSpeed",outMeasuredSpeed)
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
    bool res = CanOpenNode::configureHook();

    enableDrive();

    return res;
}

void Faulhaber3268Bx4::updateHook()
{
    double speed = 0;

    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    //mise à jour de la consigne de vitesse
    if( inSpeedCmd.readNewest(speed) == NewData )
    {
        FrontSteering_FaulhaberCommand = F_CMD_V;
        FrontSteering_FaulhaberCommandParameter = 7000*speed;
        outCommandedSpeed.write(7000*speed);
    }

    //lecture de la vitesse
    outMeasuredSpeed.write( FrontSteering_MeasuredPosition );
    //lecture du courant
    outMeasuredCurrent.write( FrontSteering_MeasuredCurrent );

    //lecture de la dernière commande envoyée :
    outLastSentCommand.write( FrontSteering_FaulHaberCommandReturn );
    outLastSentCommandParam.write( FrontSteering_FaulHaberCommandReturnParameter );
    outLastSentCommandReturn.write( FrontSteering_FaulHaberCommandReturnCode );

    attrState = ArdDs402::getStateFromCanStatusWord( FrontSteering_Ds402State );
}

void Faulhaber3268Bx4::stopHook()
{
    disableDrive();
    CanOpenNode::stopHook();
}

void Faulhaber3268Bx4::ooSendSpeed(int speed)
{
    FrontSteering_FaulhaberCommand = F_CMD_V;
    FrontSteering_FaulhaberCommandParameter = 7000*speed;
    outCommandedSpeed.write(7000*speed);
}

void Faulhaber3268Bx4::ooReadSpeed()
{
    int receivedData;
    CanDicoEntry speedMeasureSdo =  CanDicoEntry(0x21,0x6069, 0, 0, 0, 4);
    m_coReadInRemoteDico(speedMeasureSdo,&receivedData);
    outMeasuredSpeed.write(receivedData);
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
    FrontSteering_FaulhaberCommand = F_CMD_EN;
    FrontSteering_FaulhaberCommandParameter = 0;
    //TODO WLA a remplacer par un check du resultat precedent
    usleep(1000*100);
    return true;
}

bool Faulhaber3268Bx4::disableDrive()
{
    FrontSteering_FaulhaberCommand = F_CMD_DI;
    FrontSteering_FaulhaberCommandParameter = 0;
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
