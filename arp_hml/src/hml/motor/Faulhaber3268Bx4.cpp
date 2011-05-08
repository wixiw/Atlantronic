/*
 * Faulhaber3268Bx4.cpp
 *
 *  Created on: 10 mars 2011
 *      Author: wla
 */

#include "Faulhaber3268Bx4.hpp"
#include <ocl/Component.hpp>
#include "hml/can/dictionnary/CanARD.h"
#include "hml/can/wrappers/CanARDDictionnaryAccessor.hpp"
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;

ORO_LIST_COMPONENT_TYPE( arp_hml::Faulhaber3268Bx4 )

Faulhaber3268Bx4::Faulhaber3268Bx4(const std::string& name) :
        CanOpenNode(name),
        ArdMotorItf(),
        attrState(ArdDs402::UnknownDs402State),
        propInvertDriveDirection(false),
        propReductorValue(14),
        propEncoderResolution(3000),
        m_faulhaberCommandTodo(false),
        m_oldPositionMeasure(0)
{
    addAttribute("attrState",attrState);
    addAttribute("attrCommandedSpeed",attrCommandedSpeed);
    addAttribute("attrPeriod",attrPeriod);

    addProperty("propInvertDriveDirection",propInvertDriveDirection);
    addProperty("propReductorValue",propReductorValue);
    addProperty("propEncoderResolution",propEncoderResolution);

    addPort("inSpeedCmd",inSpeedCmd)
            .doc("");
    addPort("inPositionCmd",inPositionCmd)
            .doc("");
    addPort("inTorqueCmd",inTorqueCmd)
            .doc("");

    addPort("outMeasuredPosition",outMeasuredPosition)
        .doc("");
    addPort("outMeasuredTorque",outMeasuredTorque)
        .doc("");
    addPort("outComputedSpeed",outComputedSpeed)
        .doc("");
    addPort("outLastSentCommand",outLastSentCommand)
        .doc("");
    addPort("outLastSentCommandParam",outLastSentCommandParam)
        .doc("");
    addPort("outLastSentCommandReturn",outLastSentCommandReturn)
        .doc("");
    addPort("outDriveEnable",outDriveEnable)
        .doc("");
    addPort("outCurrentOperationMode",outCurrentOperationMode)
    	.doc("");

    addOperation("ooEnableDrive", &Faulhaber3268Bx4::enableDrive,this, OwnThread )
        .doc("");
    addOperation("ooDisableDrive", &Faulhaber3268Bx4::disableDrive,this, OwnThread )
        .doc("");
    addOperation("ooFaulhaberCmd", &Faulhaber3268Bx4::ooFaulhaberCmd,this, OwnThread )
        .doc("");
    addOperation("ooSetOperationMode", &Faulhaber3268Bx4::ooSetOperationMode,this, OwnThread )
    	.doc("")
    	.arg("mode"," string = speed,position,torque,homing,faulhaber");
    addOperation("coWaitEnable", &Faulhaber3268Bx4::coWaitEnable,this, ClientThread )
    	.doc("")
    	.arg("timeout","in s");

    m_measuredPosition = CanARDDictionnaryAccessor::getINTEGER32Pointer(name,"MeasuredPosition");
    m_measuredCurrent = CanARDDictionnaryAccessor::getINTEGER16Pointer(name,"MeasuredCurrent");
    m_faulhaberCommand = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommand");
    m_faulhaberCommandParameter = CanARDDictionnaryAccessor::getUNS32Pointer(name,"FaulhaberCommandParameter");
    m_faulhaberCommandReturn = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommandReturn");
    m_faulhaberCommandReturnCode = CanARDDictionnaryAccessor::getUNS8Pointer(name,"FaulhaberCommandReturnCode");
    m_faulhaberCommandReturnParameter = CanARDDictionnaryAccessor::getUNS32Pointer(name,"FaulhaberCommandReturnParameter");
    m_ds402State = CanARDDictionnaryAccessor::getUNS16Pointer(name,"Ds402State");

    ArdMotorItf::setOperationMode(ArdMotorItf::SPEED_CONTROL);
    outDriveEnable.write(false);
    clock_gettime(CLOCK_MONOTONIC, &m_oldPositionMeasureTime);
}

bool Faulhaber3268Bx4::checkInputsPorts()
{
	bool res = true;

    if( inSpeedCmd.connected() == false )
    {
    	res = false;
    	LOG(Error) << "failed to configure : inSpeedCommand is not connected" << endlog();
    }

    return res;
}

bool Faulhaber3268Bx4::configureHook()
{
	return init();
}

void Faulhaber3268Bx4::updateHook()
{
    //appel du parent car il log les bootUp
    CanOpenNode::updateHook();

    //read inputs from Orocos interface
    getInputs();

    //appel du run de la classe générique moteur
    ArdMotorItf::run();

    //get measures
    readCaptors();

    //publish to Orocos interface
    setOutputs();
}

void Faulhaber3268Bx4::getInputs()
{
    double speedCmd = 0;
    if( inSpeedCmd.readNewest(speedCmd) != NoData )
    {
    	ArdMotorItf::setSpeedCmd(speedCmd);
    }

    double positionCmd = 0;
    if( inPositionCmd.readNewest(positionCmd) != NoData )
    {
    	ArdMotorItf::setPositionCmd(positionCmd);
    }

    double torqueCmd = 0;
    if( inTorqueCmd.readNewest(torqueCmd) != NoData )
    {
    	ArdMotorItf::setTorqueCmd(torqueCmd);
    }
}

void Faulhaber3268Bx4::setOutputs()
{
	//publication de la position
	outMeasuredPosition.write( ArdMotorItf::getPositionMeasure() );
    //publication de la vitesse
	outComputedSpeed.write( ArdMotorItf::getSpeedMeasure() );
    //lecture du courant
    outMeasuredTorque.write( ArdMotorItf::getTorqueMeasure() );

    //publication du mode d'operation
    outCurrentOperationMode.write( getStringFromMode(getOperationMode()) );

    //publication de l'état enable
    if( attrState == ArdDs402::OperationEnable )
    	outDriveEnable.write(true);
    else
    	outDriveEnable.write(false);

    EnterMutex();
    //lecture de la dernière commande envoyée :
    outLastSentCommand.write( *m_faulhaberCommandReturn );
    outLastSentCommandParam.write( *m_faulhaberCommandReturnParameter );
    outLastSentCommandReturn.write( *m_faulhaberCommandReturnCode );
    LeaveMutex();
}

void Faulhaber3268Bx4::runSpeed()
{
	attrCommandedSpeed = m_speedCommand;
	//conversion de rad/s sur la roue, vers RPM sur l'axe moteur
	UNS32 speed = attrCommandedSpeed*propReductorValue*RAD_S_TO_RPM;
	//inversion de polarité soft
	if( propInvertDriveDirection )
	{
		speed = -speed;
	}

    EnterMutex();
	*m_faulhaberCommand = F_CMD_V;
	*m_faulhaberCommandParameter = speed;
    LeaveMutex();
}

void Faulhaber3268Bx4::runTorque()
{
	//TODO WLA
}

void Faulhaber3268Bx4::runPosition()
{
	//TODO WLA
}

void Faulhaber3268Bx4::runHoming()
{
	//TODO WLA
}

void Faulhaber3268Bx4::runOther()
{
	if( m_faulhaberCommandTodo )
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
			LOG(Error) << "faulhaber command " << (int) m_faulhaberScriptCommand << " failed : return code " << outLastSentCommandReturn.getLastWrittenValue() << endlog();
		}
		else
		{
			LOG(Info) << "faulhaber " << (int) m_faulhaberScriptCommand << " command succeed" << endlog();
			m_faulhaberCommandTodo = false;
		}
	}
}

void Faulhaber3268Bx4::readCaptors()
{
    EnterMutex();
    //lecture de la position
    double position = TURN_TO_RAD*(*m_measuredPosition)/(propReductorValue*propEncoderResolution);
	if( propInvertDriveDirection )
	{
		position = -position;
	}
	m_positionMeasure = position;

    //lecture de l'état DS402s
    attrState = ArdDs402::getStateFromCanStatusWord( *m_ds402State );

    //lecture du courant
    //TODO WLA mettre dans la bonne unité
	double current = *m_measuredCurrent;
	m_torqueMeasure = current;
	LeaveMutex();

	//calcul de la vitesse
	struct timespec now;
	struct timespec delay;
	clock_gettime(CLOCK_MONOTONIC, &now);
	delta_t(&delay, &m_oldPositionMeasureTime, &now);
	attrPeriod = delay.tv_sec+(double)(delay.tv_nsec)/1E9;
	m_speedMeasure = (m_positionMeasure - m_oldPositionMeasure)/attrPeriod;
	m_oldPositionMeasure = m_positionMeasure;
	m_oldPositionMeasureTime = now;
}

void Faulhaber3268Bx4::stopHook()
{
    disableDrive();
    updateHook();
    CanOpenNode::stopHook();
}

/**********************************************************************/
/*              Operation Orocos		                              */
/**********************************************************************/

void Faulhaber3268Bx4::ooFaulhaberCmd(int cmd, int param)
{
	ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
	m_faulhaberScriptCommand = (UNS8) cmd;
	m_faulhaberScriptCommandParam = (UNS32) param;
	m_faulhaberCommandTodo = true;
}

bool Faulhaber3268Bx4::ooSetOperationMode(std::string mode)
{
	bool res = false;

	if( setOperationMode(getModeFromString(mode)) )
	{
		LOG(Info) << "switch to " << mode <<" mode" << endl;
		res = true;
	}
	else
	{
		LOG(Error) << "Failed to switch to " << mode << " operation mode. Checks syntax or required mode is not available" << endlog();
		res = false;
	}

	return res;
}

bool Faulhaber3268Bx4::coWaitEnable(double timeout)
{
	double chrono = 0;
	bool res = false;

    while( outDriveEnable.getLastWrittenValue()==false && chrono < timeout )
    {
    	LOG(Debug) << "coWaitEnable : is waiting for enable to come" << endlog();
        chrono+=0.050;
        usleep(50*1000);
    }
    //si le timeout est explosé c'est que ça a foiré
    if( chrono >= timeout )
    {
        LOG(Error)  << "coWaitEnable : timeout has expired, Drive is not enabled !"<< endlog();
        res = false;
    }
    else
    	res = true;

    return res;
}


/**********************************************************************/
/*              Interface moteur générique                            */
/**********************************************************************/

bool Faulhaber3268Bx4::setOperationMode(ArdMotorItf::operationMode_t operationMode)
{
	bool res = true;

	//pour le moment les seuls modes accessibles sont speed et other=faulhaber command
	if( ArdMotorItf::SPEED_CONTROL == operationMode || ArdMotorItf::OTHER == operationMode)
		ArdMotorItf::setOperationMode(operationMode);
	else
		res = false;

	return res;
}



bool Faulhaber3268Bx4::init()
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

void Faulhaber3268Bx4::enableDrive()
{
	ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
	m_faulhaberScriptCommand = F_CMD_EN;
	m_faulhaberScriptCommandParam = 0;
	m_faulhaberCommandTodo = true;
}

void Faulhaber3268Bx4::disableDrive()
{
	ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
	m_faulhaberScriptCommand = F_CMD_DI;
	m_faulhaberScriptCommandParam = 0;
	m_faulhaberCommandTodo = true;
}


bool Faulhaber3268Bx4::reset()
{
	return CanOpenNode::resetNode();
}

bool Faulhaber3268Bx4::getLimitSwitchStatus()
{
	//there is no limit switch currently
    return false;
}

bool Faulhaber3268Bx4::startWatchdog()
{
	//there is no watchdog yet
    return false;
}

bool Faulhaber3268Bx4::stopWatchdog()
{
	//there is no watchdog yet
    return false;
}

//TODO WLA
bool Faulhaber3268Bx4::isInError()
{
    return false;
}

//TODO WLA
unsigned int Faulhaber3268Bx4::getError()
{
    return 0;
}

void Faulhaber3268Bx4::delta_t(struct timespec *interval, struct timespec *begin, struct timespec *now)
{
	interval->tv_nsec = now->tv_nsec - begin->tv_nsec; /* Subtract 'decimal fraction' first */
	if(interval->tv_nsec < 0 )
	{
		interval->tv_nsec += 1000000000; /* Borrow 1sec from 'tv_sec' if subtraction -ve */
		interval->tv_sec = now->tv_sec - begin->tv_sec - 1; /* Subtract whole number of seconds and return 1 */
	}
	else
	{
		interval->tv_sec = now->tv_sec - begin->tv_sec; /* Subtract whole number of seconds and return 0 */
	}
}
