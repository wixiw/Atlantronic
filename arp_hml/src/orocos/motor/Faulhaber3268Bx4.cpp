/*
 * Faulhaber3268Bx4.cpp
 *
 *  Created on: 10 mars 2011
 *      Author: wla
 */

#include "Faulhaber3268Bx4.hpp"
#include <rtt/Component.hpp>
#include "orocos/can/wrappers/CanARDDictionnaryAccessor.hpp"
#include <math/math.hpp>

using namespace arp_hml;
using namespace arp_core;
using namespace arp_math;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_hml::Faulhaber3268Bx4 )

Faulhaber3268Bx4::Faulhaber3268Bx4(const std::string& name) :
        CanOpenNode(name),
        ArdMotorItf(),
        attrState(ArdDs402::UnknownDs402State),
        attrBlockingDelay(0),
        attrHomingState(NOT_IN_HOMNG_MODE),
        attrHardNotify(false),
        propInvertDriveDirection(false),
        propReductorValue(3969.0 / 289.0),
        propEncoderResolution(3000),
        propMaximalTorque(0.5),
        propInputsTimeout(1.0),
        propHomingSpeed(4),
        propBlockingTorqueTimeout(0.500),
        m_faulhaberCommandTodo(false),
        m_oldPositionMeasure(0),
        m_isMotorBlocked(false)
{
    createOrocosInterface();

    m_measuredPosition = CanARDDictionnaryAccessor::getINTEGER32Pointer(name,"MeasuredPosition");
    m_measuredCurrent = CanARDDictionnaryAccessor::getINTEGER16Pointer(name,"MeasuredCurrent");
    m_measuredPeriod = CanARDDictionnaryAccessor::getUNS8Pointer(name,"TimeCode");
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

bool Faulhaber3268Bx4::checkProperties()
{
	bool res = CanOpenNode::checkProperties();

	if( propReductorValue <= 1 || propReductorValue > 1000 )
	{
    	res = false;
    	LOG(Error) << "checkProperties : propReductorValue has an incorrect value should be in ]1;1000] current:" << propReductorValue << endlog();
	}

	if( propEncoderResolution <= 1 || propEncoderResolution > 10000 )
	{
    	res = false;
    	LOG(Error) << "checkProperties : propEncoderResolution has an incorrect value should be in ]1;10000] point/rev current:" << propEncoderResolution << endlog();
	}

    if( propMaximalTorque <= 0.2 || propMaximalTorque > 11 )
    {
        res = false;
        LOG(Error) << "checkProperties : propMaximalTorque has an incorrect value should be in ]0.2;11.0] AMpscurrent:" << propMaximalTorque << endlog();
    }

    return res;
}

bool Faulhaber3268Bx4::configureHook()
{
    int numberOfPdo = 0 ;
    int i = 0;

    if( !init() )
        goto fail;

    if( !CanOpenNode::configureHook() )
        goto fail;


    //on recherche dans la liste des PDO l'index du PDO de la commande Faulhaber pour pouvoir l'envoyer directement dans certains modes de fonctionnement
    if( CanARD_Data.lastIndex == NULL || CanARD_Data.firstIndex == NULL )
    {
        LOG(Error) << "The Dictionnary seems to be corrupted ! Index reference are NULL." << endlog();
        goto fail;
    }
    numberOfPdo = CanARD_Data.lastIndex->PDO_TRS_MAP - CanARD_Data.firstIndex->PDO_TRS_MAP;
    if( numberOfPdo <= 0 )
    {
        LOG(Error) << "The Dictionnary seems to be corrupted ! Number of PDO should be positive." << endlog();
        goto fail;
    }
    for( i = 0 ; i< numberOfPdo ; i++ )
    {
        //cout << "tested PDO in dico : [" << i << "] 0x" << std::hex << CanARD_Data.PDO_status[i].last_message.cob_id << std::dec << endl;
        if( CanARD_Data.PDO_status[i].last_message.cob_id == 0x300 + propNodeId )
        {
            attrFaulhaberCommandPdoIndex = i;
            break;
        }
    }
    //si on est là, on n'a pas trouvé.
    if( i == numberOfPdo )
    {
        LOG(Error) << "Failed to find Faulhaber Command PDO number among " << numberOfPdo << " tested index. Check your dictionnary." << endlog();
        goto fail;
    }

    goto success;

    fail:
	    return false;
	success:
	    return true;
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
	//read last speed command
    double speedCmd = 0;
    if( inSpeedCmd.read(speedCmd) == NewData )
    {
    	ArdMotorItf::setSpeedCmd(speedCmd);
    	m_oldSpeedCommandTime = attrSyncTime;
    }
    //if we did not get a speed command since a time, we assume a 0 cmd for security reasons
    else if(arp_math::delta_t(m_oldSpeedCommandTime,attrSyncTime) > propInputsTimeout)
    {
        ArdMotorItf::setSpeedCmd(0);
    }

    //read last position command
    double positionCmd = 0;
    if( inPositionCmd.readNewest(positionCmd) != NoData )
    {
    	ArdMotorItf::setPositionCmd(positionCmd);
    }

    //read last torque command
    double torqueCmd = 0;
    if( inTorqueCmd.read(torqueCmd) == NewData )
    {
    	ArdMotorItf::setTorqueCmd(torqueCmd);
    	m_oldTorqueCommandTime = attrSyncTime;
    }
    //if we did not get a speed command since a time, we assume a 0 cmd for security reasons
    else if(arp_math::delta_t(m_oldTorqueCommandTime,attrSyncTime) > propInputsTimeout)
    {
        ArdMotorItf::setTorqueCmd(0);
    }
}

void Faulhaber3268Bx4::setOutputs()
{
	//publication de la position
	outPosition.write( ArdMotorItf::getPositionMeasure() );

    //publication de la vitesse
	outVelocity.write( ArdMotorItf::getSpeedMeasure() );
    //lecture du courant
    outTorque.write( ArdMotorItf::getTorqueMeasure() );

    //publication du blocage
    if( m_isMotorBlocked && attrBlockingDelay >= propBlockingTorqueTimeout )
        outMaxTorqueTimeout.write( true );
    else
        outMaxTorqueTimeout.write(false);

    //publication du mode d'operation
    outCurrentOperationMode.write( getStringFromMode(getOperationMode()) );

    //publication de l'état enable
    if( attrState == ArdDs402::OperationEnable )
    	outDriveEnable.write(true);
    else
    	outDriveEnable.write(false);

    //publication de la fin de sequence homing
    if( attrHomingState == HOMING_DONE && getOperationMode() == HOMING )
    {
        if( outHomingDone.getLastWrittenValue() == false )
            LOG(Info) << "Homing Done" << endlog();
        outHomingDone.write(true);
    }
    else
        outHomingDone.write(false);

    EnterMutex();
    //lecture de la dernière commande envoyée :
    outLastSentCommand.write( *m_faulhaberCommandReturn );
    outLastSentCommandParam.write( *m_faulhaberCommandReturnParameter );
    outLastSentCommandReturn.write( *m_faulhaberCommandReturnCode );
    LeaveMutex();

    //a publier quand on a finit.
    outClock.write( attrSyncTime );
}

void Faulhaber3268Bx4::runSpeed()
{
    // The submodes are not called is the motor is not powered
    if( outDriveEnable.getLastWrittenValue() )
    {
        //conversion de rad/s sur la roue, vers RPM sur l'axe moteur
        UNS32 speed = m_speedCommand*propReductorValue*RAD_S_TO_RPM;

        //inversion de polarité soft
        if( propInvertDriveDirection )
        {
            speed = -speed;
        }

        EnterMutex();
        *m_faulhaberCommand = F_CMD_V;
        *m_faulhaberCommandParameter = speed;
        LeaveMutex();
    /*
        usleep(500);

        //TODO gain de conversion...
        UNS32 torque = m_torqueCommand/propReductorValue*NM_TO_A*1000;
        EnterMutex();
        *m_faulhaberCommand = F_CMD_LPC;
        *m_faulhaberCommandParameter = torque;
        LeaveMutex();*/
    }
}

void Faulhaber3268Bx4::runTorque()
{
    // The submodes are not called is the motor is not powered
    if( outDriveEnable.getLastWrittenValue() )
    {
        //TODO WLA Faulhaber3268Bx4 implémenter control en torque
    }
}

void Faulhaber3268Bx4::runPosition()
{
    // The submodes are not called is the motor is not powered
    if( outDriveEnable.getLastWrittenValue() )
    {
        //conversion de rad sur la roue, vers des tours sur l'axe moteur
        UNS32 position = m_positionCommand*propReductorValue*RAD_TO_TURN*(double)propEncoderResolution;

        //inversion de polarité soft
        if( propInvertDriveDirection )
        {
            position = -position;
        }

        EnterMutex();
        *m_faulhaberCommand = F_CMD_LA;
        *m_faulhaberCommandParameter = position;
        LeaveMutex();
        coSendPdo(attrFaulhaberCommandPdoIndex);

        EnterMutex();
        *m_faulhaberCommand = F_CMD_M;
        *m_faulhaberCommandParameter = 0;
        LeaveMutex();
    }
}

void Faulhaber3268Bx4::runHoming()
{
    //TODO gestion de homing error ? ...
    // The submodes are not called is the motor is not powered
    if( outDriveEnable.getLastWrittenValue() )
    {
        switch (attrHomingState) {
            case ASK_CONFIGURE_EDGE:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_HP;
                    m_faulhaberScriptCommandParam = (UNS32) F_CMD_HP_FAILING_EDGE;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_CONFIGURE_EDGE;
                }
                break;
            case WAIT_CONFIGURE_EDGE:
                if( !m_faulhaberCommandTodo )
                 {
                     attrHomingState = ASK_CONFIGURE_SWITCH1;
                 }
                break;
            case ASK_CONFIGURE_SWITCH1:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_SHL;
                    m_faulhaberScriptCommandParam = F_ANALOG_SWICTH_MASK;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_CONFIGURE_SWITCH1;
                }
                break;
            case WAIT_CONFIGURE_SWITCH1:
                if( !m_faulhaberCommandTodo )
                 {
                     attrHomingState = ASK_CONFIGURE_SWITCH2;
                 }
                break;
            case ASK_CONFIGURE_SWITCH2:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_SHN;
                    m_faulhaberScriptCommandParam = F_ANALOG_SWICTH_MASK;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_CONFIGURE_SWITCH2;
                }
                break;
            case WAIT_CONFIGURE_SWITCH2:
                if( !m_faulhaberCommandTodo )
                 {
                     attrHomingState = ASK_CONFIGURE_SWITCH3;
                 }
                break;
            case ASK_CONFIGURE_SWITCH3:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_SHA;
                    m_faulhaberScriptCommandParam = F_ANALOG_SWICTH_MASK;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_CONFIGURE_SWITCH3;
                }
                break;
            case WAIT_CONFIGURE_SWITCH3:
                if( !m_faulhaberCommandTodo )
                 {
                     attrHomingState = ASK_CONFIGURE_SPEED;
                 }
                break;
            case ASK_CONFIGURE_SPEED:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_HOSP;
                    m_faulhaberScriptCommandParam = (UNS32) (propHomingSpeed*propReductorValue*RAD_S_TO_RPM);
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_CONFIGURE_SPEED;
                }
                break;
            case WAIT_CONFIGURE_SPEED:
                if( !m_faulhaberCommandTodo )
                 {
                     attrHomingState = ASK_HOMING;
                 }
                break;
            case ASK_HOMING:
                if( !m_faulhaberCommandTodo )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_GOHOSEQ;
                    m_faulhaberScriptCommandParam = (UNS32) 0;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = WAIT_HOMING;
                }
                break;
            case WAIT_HOMING:
                if( attrHardNotify )
                {
                    m_faulhaberScriptCommand = (UNS8) F_CMD_V;
                    m_faulhaberScriptCommandParam = (UNS32) 0;
                    m_faulhaberCommandTodo = true;
                    attrHomingState = SET_NULL_SPEED;
                }
                break;
            case SET_NULL_SPEED:
                if( !m_faulhaberCommandTodo )
                {
                    attrHomingState = HOMING_DONE;
                }
                break;
            case HOMING_DONE:
                break;
            default:
                LOG(Error) << "Unknow homing state : " << attrHomingState << endlog();
                return;
                break;
        }

        //en fait on pilote le mode homing en mode Faulhaber
        runOther();
    }
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
			LOG(Error) << "faulhaber command 0x" << std::hex << (int) m_faulhaberScriptCommand << " failed : return code " << std::dec << outLastSentCommandReturn.getLastWrittenValue() << endlog();
		}
		else
		{
			LOG(Info) << "faulhaber 0x"  << std::hex <<  (int) m_faulhaberScriptCommand << " command succeed with return " << std::hex << outLastSentCommandReturn.getLastWrittenValue() << endlog();
			m_faulhaberCommandTodo = false;
		}
	}
}

void Faulhaber3268Bx4::readCaptors()
{
    EnterMutex();
    //lecture de la position
    attrIncrementalOdometer = (*m_measuredPosition);
    double position = TURN_TO_RAD*(double)attrIncrementalOdometer/(propReductorValue*(double)propEncoderResolution);
	if( propInvertDriveDirection )
	{
		position = -position;
	}
	m_positionMeasure = position;

    //lecture de l'état DS402s
    attrState = ArdDs402::getStateFromCanStatusWord( *m_ds402State );
    outStateBrut.write(*m_ds402State);
    attrHardNotify = *m_ds402State&=(1<<14);

    //lecture du courant (en mA)
	double current = *m_measuredCurrent;
	m_torqueMeasure = current/1000;

    //lecture de la période (en ms) à enregistrer dans attribut en s
	//ATTENTION : on passe outre le port inMasterPeriod
	//TODO gerer le cas 0xFF
    attrPeriod = (double)(*m_measuredPeriod)/1000.0;
	LeaveMutex();

	//calcul de la vitesse
	if( attrPeriod > 0 )
	{
	    m_speedMeasure = (m_positionMeasure - m_oldPositionMeasure)/attrPeriod;
	}
	else
	{
	    m_speedMeasure = 0;
	}
	m_oldPositionMeasure = m_positionMeasure;
	m_oldPositionMeasureTime = attrSyncTime;

	//détection du blocage
	//*0.95 certainement parce que le moteur étant limité hardwarement à propMaximalTorque,
	//on ne peut pas le dépasser.
	if( fabs(ArdMotorItf::getTorqueMeasure()) >= propMaximalTorque*0.95 )
	{
	    attrBlockingDelay += attrPeriod;
	    m_isMotorBlocked = true;
	}
	else
	{
	    m_isMotorBlocked = false;
	    attrBlockingDelay = 0;
	}
}

void Faulhaber3268Bx4::stopHook()
{
    EnterMutex();
    *m_faulhaberCommand = m_faulhaberScriptCommand;
    *m_faulhaberCommandParameter = m_faulhaberScriptCommandParam;
    LeaveMutex();
    CanOpenNode::stopHook();
}

/**********************************************************************/
/*              Operation Orocos		                              */
/**********************************************************************/

bool Faulhaber3268Bx4::ooLimitCurrent(double ampValue)
{
    if( outDriveEnable.getLastWrittenValue() )
    {
        LOG(Error) << "Failed to limitCurrent because the drive are not disabled (this limitation is not HW dependent, it's an ARD choice)" << endlog();
        return false;
    }

    if( !isRunning() )
    {
        LOG(Error) << "Failed to limitCurrent because the component is not running" << endlog();
        return false;
    }

    ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
    m_faulhaberScriptCommand = F_CMD_LPC;
    m_faulhaberScriptCommandParam = (UNS32) ampValue*1000;
    m_faulhaberCommandTodo = true;
    return true;
}

void Faulhaber3268Bx4::ooFaulhaberCmd(int cmd, int param)
{
	ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
	m_faulhaberScriptCommand = (UNS8) cmd;
	m_faulhaberScriptCommandParam = (UNS32) param;
	m_faulhaberCommandTodo = true;
}

void Faulhaber3268Bx4::ooSleep(int dt)
{
    sleep(dt);
}

bool Faulhaber3268Bx4::ooSetOperationMode(std::string mode)
{
	bool res = false;

	//This operation is only accessible when the component is running
	if( isRunning() )
	{
		if( setOperationMode(getModeFromString(mode)) )
		{
			LOG(Info) << "switch to " << mode <<" mode" << endlog();
			res = true;
		}
		else
		{
			LOG(Error) << "Failed to switch to " << mode << " operation mode. Check syntax or required mode is not available" << endlog();
			res = false;
		}
	}
	else
	{
		LOG(Error) << "Failed to switch to " << mode << " operation mode. The motor is not in running orocos state" << endlog();
		res = false;
	}

	return res;
}

bool Faulhaber3268Bx4::coWaitEnable(double timeout)
{
	double chrono = 0;
	bool res = false;

	//This operation is only accessible when the component is running
	if( isRunning() )
	{
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
	}
	else
	{
		LOG(Error) << "Can not wait Enable mode. The motor is not in running orocos state" << endlog();
		res = false;
	}

    return res;
}


/**********************************************************************/
/*              Interface moteur générique                            */
/**********************************************************************/

bool Faulhaber3268Bx4::setOperationMode(ArdMotorItf::operationMode_t operationMode)
{
	bool res = true;

	//This operation is only accessible when the component is running
	if( isRunning() )
	{
		//pour le moment les seuls modes accessibles sont speed et other=faulhaber command
		if( ArdMotorItf::SPEED_CONTROL == operationMode
		        || ArdMotorItf::OTHER == operationMode
		        || ArdMotorItf::POSITION_CONTROL == operationMode
		        || ArdMotorItf::HOMING == operationMode)
		{
		    //initialisation de l'état homing
		    if( ArdMotorItf::HOMING == operationMode )
		    {
		        attrHomingState = ASK_CONFIGURE_EDGE;
		        attrHardNotify = false;
		    }
		    //desinit de l'état homing
		    else
		    {
		        attrHomingState = NOT_IN_HOMNG_MODE;
		        attrHardNotify = false;
		    }

			ArdMotorItf::setOperationMode(operationMode);
		}
		else
		{
		    LOG(Error) << "This mode is not available for now" << endlog();
			res = false;
		}
	}
	else
	{
		LOG(Error) << "Failed to setOperationMode mode. The motor is not in running orocos state" << endlog();
		res = false;
	}

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

    return res;
}

void Faulhaber3268Bx4::enableDrive()
{
    if( !isRunning() )
    {
        LOG(Error) << "Failed to limitCurrent because the component is not running" << endlog();
    }
    else
    {
        ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
        m_faulhaberScriptCommand = F_CMD_EN;
        m_faulhaberScriptCommandParam = 0;
        m_faulhaberCommandTodo = true;
    }
}

void Faulhaber3268Bx4::disableDrive()
{
    if( !isRunning() )
    {
        LOG(Error) << "Failed to limitCurrent because the component is not running" << endlog();
    }
    else
    {
        ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
        m_faulhaberScriptCommand = F_CMD_DI;
        m_faulhaberScriptCommandParam = 0;
        m_faulhaberCommandTodo = true;
    }
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

bool Faulhaber3268Bx4::isInError()
{
    return false;
}

unsigned int Faulhaber3268Bx4::getError()
{
    return 0;
}

void Faulhaber3268Bx4::createOrocosInterface()
{
    addAttribute("attrState",attrState);
    addAttribute("attrVelocityCmd",m_speedCommand);
    addAttribute("attrPositionCmd",m_positionCommand);
    addAttribute("attrTorqueCmd",m_torqueCommand);
    addAttribute("attrBlockingDelay",attrBlockingDelay);
    addAttribute("attrIncrementalOdometer",attrIncrementalOdometer);
    addAttribute("attrFaulhaberCommandPdoIndex",attrFaulhaberCommandPdoIndex);
    addAttribute("attrHomingState",attrHomingState);

    addProperty("propInvertDriveDirection",propInvertDriveDirection)
        .doc("Is true when you when to invert the speed command and feedback of the motor softly");
    addProperty("propReductorValue",propReductorValue)
        .doc("Reductor's value from the motor's axe to the reductor's output's axe. So it should be greather than 1");
    addProperty("propEncoderResolution",propEncoderResolution)
        .doc("Encoder resolution in point by rev");
    addProperty("propMaximalTorque",propMaximalTorque)
        .doc("Maximal Torque allowed in Amps");
    addProperty("propInputsTimeout",propInputsTimeout)
            .doc("Maximal delay beetween 2 commands to consider someone is still giving coherent orders, in s");
    addProperty("propHomingSpeed",propHomingSpeed)
            .doc("Homing speed in rad/s on the reductor output");
    addProperty("propBlockingTorqueTimeout",propBlockingTorqueTimeout)
            .doc("Timeout before thinking a wheel is blocked. See outMaxTorqueTimeout");

    addPort("inSpeedCmd",inSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");
    addPort("inPositionCmd",inPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");

    addPort("inTorqueCmd",inTorqueCmd)
            .doc("Command to be used in torque mode. This mode is not available yes");

    addPort("outStateBrut",outStateBrut)
        .doc("Debug port to see the motor CAN state value");
    addPort("outPosition",outPosition)
        .doc("Provides the measured position of the encoder from CAN. It is converted in rad on the reductor's output's axe.");
    addPort("outClock",outClock)
        .doc("");
    addPort("outTorque",outTorque)
        .doc("Provides the torque measured from CAN. Not available yet");
    addPort("outVelocity",outVelocity)
        .doc(" Provides a computed speed from the encoder position. In rad/s on the reductor's output's axe.");
    addPort("outLastSentCommand",outLastSentCommand)
        .doc("Prints the last Faulhaber command sent on CAN in OTHER mode of operation");
    addPort("outLastSentCommandParam",outLastSentCommandParam)
        .doc("Prints the last Faulhaber params sent on CAN in OTHER mode of operation");
    addPort("outLastSentCommandReturn",outLastSentCommandReturn)
        .doc("Prints the last Faulhaber command return received from CAN in OTHER mode of operation");
    addPort("outDriveEnable",outDriveEnable)
        .doc("Is true when the drive is ready to be operated (axe blocked). If it is false, the axe is free of any mouvement");
    addPort("outCurrentOperationMode",outCurrentOperationMode)
        .doc("Provides the current mode of operation of the motor (speed,position,torque,homing,other=faulhaber)");
    addPort("outMaxTorqueTimeout",outMaxTorqueTimeout)
        .doc(" Is true when the propMaximalTorque has been reached (at 95%)for more propBlockingTorqueTimeout");
    addPort("outHomingDone",outHomingDone)
        .doc("Is true when the Homing sequence is finished. It has no sense when the motor is not in HOMING mode");

    addOperation("ooEnableDrive", &Faulhaber3268Bx4::enableDrive,this, OwnThread )
        .doc("Activate the power on the motor. Without a speed command it acts as a brake. Returns false if the component is not running.");
    addOperation("ooDisableDrive", &Faulhaber3268Bx4::disableDrive,this, OwnThread )
        .doc("disable power on the motor, this is a freewheel mode. Returns false if the component is not running.");
    addOperation("ooLimitCurrent", &Faulhaber3268Bx4::ooLimitCurrent,this, OwnThread )
        .doc("Limit the current given to the motor. Should be greather then 0.2A and lower than 10A. Returns false if the motor are not disabled or the component is not running.")
        .arg("currentValue","in A");
    addOperation("ooFaulhaberCmd", &Faulhaber3268Bx4::ooFaulhaberCmd,this, OwnThread )
        .doc("");
    addOperation("ooSetOperationMode", &Faulhaber3268Bx4::ooSetOperationMode,this, OwnThread )
        .doc("")
        .arg("mode"," string = speed,position,torque,homing,faulhaber");
    addOperation("ooSleep", &Faulhaber3268Bx4::ooSleep,this, ClientThread )
           .doc("Permet d'attendre pour bloquer le script de déploiement")
           .arg("dt"," temps à dormir en s");
    addOperation("coWaitEnable", &Faulhaber3268Bx4::coWaitEnable,this, ClientThread )
        .doc("")
        .arg("timeout","in s");
}
