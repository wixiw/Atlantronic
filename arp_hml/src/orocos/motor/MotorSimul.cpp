/*
 * MotorSimul.cpp
 *
 *  Created on: 02 April 2012
 *      Author: wla
 */

#include "MotorSimul.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_hml;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_hml::MotorSimul )

MotorSimul::MotorSimul(const std::string& name):
    HmlTaskContext(name),
    ArdMotorItf(),
    attrBlockingDelay(0),
    propReductorValue(14),
    propEncoderResolution(3000),
    propMaximalTorque(0.5),
    propInputsTimeout(20.0),
    propHomingSpeed(700),
    m_power(false)

{
    addAttribute("attrCommandedSpeed",m_speedCommand);
    addAttribute("attrPositionCmd",m_positionCommand);
    addAttribute("attrTorqueCmd",m_torqueCommand);
    addAttribute("attrPeriod",attrPeriod);
    addAttribute("attrBlockingDelay",attrBlockingDelay);
    addAttribute("attrIncrementalOdometer",attrIncrementalOdometer);

    addProperty("propReductorValue",propReductorValue)
        .doc("Reductor's value from the motor's axe to the reductor's output's axe. So it should be greather than 1");
    addProperty("propEncoderResolution",propEncoderResolution)
        .doc("Encoder resolution in point by rev");
    addProperty("propMaximalTorque",propMaximalTorque)
        .doc("Maximal Torque allowed in Amps");
    addProperty("propInputsTimeout",propInputsTimeout)
            .doc("Maximal delay beetween 2 commands to consider someone is still giving coherent orders, in s");
    addProperty("propHomingSpeed",propHomingSpeed)
            .doc("");


    addPort("inClock",inClock)
            .doc("Port to fire to wake up the component");
    addPort("inSpeedCmd",inSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");
    addPort("inPositionCmd",inPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");
    addPort("inTorqueCmd",inTorqueCmd)
            .doc("Command to be used in torque mode. This mode is not available yes");
    addPort("inBlockMotor",inBlockMotor)
                .doc("order from outside to block the motor");

    addPort("outFilteredSpeedCommand",outFilteredSpeedCommand)
            .doc("");
    addPort("outFilteredPositionCommand",outFilteredPositionCommand)
            .doc("");

    addPort("outPosition",outPosition)
        .doc("Provides the measured position of the encoder from CAN. It is converted in rad on the reductor's output's axe.");
    addPort("outClock",outClock)
        .doc("");
    addPort("outTorque",outTorque)
        .doc("Provides the torque measured from CAN. Not available yet");
    addPort("outVelocity",outVelocity)
        .doc(" Provides a computed speed from the encoder position. In rad/s on the reductor's output's axe.");
    addPort("outDriveEnable",outDriveEnable)
        .doc("Is true when the drive is ready to be operated (axe blocked). If it is false, the axe is free of any mouvement");
    addPort("outCurrentOperationMode",outCurrentOperationMode)
        .doc("Provides the current mode of operation of the motor (speed,position,torque,homing,other=faulhaber)");
    addPort("outMaxTorqueTimeout",outMaxTorqueTimeout)
        .doc(" Is true when the propMaximalTorque has been reached for more propBlockingTorqueTimeout");
    addPort("outConnected",outConnected)
        .doc("Always true");
    addPort("outHomingDone",outHomingDone)
        .doc("Always true is Homing sequence under power, false in any other case");

    addOperation("ooEnableDrive", &MotorSimul::enableDrive,this, OwnThread )
        .doc("Activate the power on the motor. Without a speed command it acts as a brake. Returns false if the component is not running.");
    addOperation("ooDisableDrive", &MotorSimul::disableDrive,this, OwnThread )
        .doc("disable power on the motor, this is a freewheel mode. Returns false if the component is not running.");
    addOperation("ooLimitCurrent", &MotorSimul::ooLimitCurrent,this, OwnThread )
        .doc("Limit the current given to the motor. Should be greather then 0.2A and lower than 10A. Returns false if the motor are not disabled or the component is not running.")
        .arg("currentValue","in A");
    addOperation("ooFaulhaberCmd", &MotorSimul::ooFaulhaberCmd,this, OwnThread )
        .doc("");
    addOperation("ooSetOperationMode", &MotorSimul::ooSetOperationMode,this, OwnThread )
        .doc("")
        .arg("mode"," string = speed,position,torque,homing,faulhaber");
    addOperation("ooSleep", &MotorSimul::ooSleep,this, ClientThread )
           .doc("Permet d'attendre pour bloquer le script de déploiement")
           .arg("dt"," temps à dormir en s");
    addOperation("coWaitEnable", &MotorSimul::coWaitEnable,this, ClientThread )
        .doc("")
        .arg("timeout","in s");
}

bool MotorSimul::configureHook()
{
    bool res = HmlTaskContext::configureHook();
    return res;
}

void MotorSimul::updateHook()
{
    getInputs();

    //appel du run de la classe générique moteur
    ArdMotorItf::run();

    setOutputs();
}

void MotorSimul::getInputs()
{
    //Récupération de la date du cycle CAN
    inClock.readNewest(m_syncTime);

    inBlockMotor.readNewest(m_blockMotor);

    //read last speed command
    double speedCmd = 0;
    if( inSpeedCmd.read(speedCmd) == NewData )
    {
        ArdMotorItf::setSpeedCmd(speedCmd);
        m_oldSpeedCommandTime = m_syncTime;
    }
    //if we did not get a speed command since a time, we assume a 0 cmd for security reasons
    else if( arp_math::delta_t(m_oldSpeedCommandTime,m_syncTime) > propInputsTimeout)
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
        m_oldTorqueCommandTime = m_syncTime;
    }
    //if we did not get a speed command since a time, we assume a 0 cmd for security reasons
    else if( arp_math::delta_t(m_oldTorqueCommandTime, m_syncTime ) > propInputsTimeout)
    {
        ArdMotorItf::setTorqueCmd(0);
    }
}


void MotorSimul::setOutputs()
{
    //publication de la position
    outPosition.write( ArdMotorItf::getPositionMeasure() );
    //publication de la vitesse
    outVelocity.write( ArdMotorItf::getSpeedMeasure() );
    //lecture du courant
    outTorque.write( ArdMotorItf::getTorqueMeasure() );
    //publication du mode d'operation
    outCurrentOperationMode.write( getStringFromMode(getOperationMode()) );

    outConnected.write(true);
    outDriveEnable.write(m_power);

    //a faire a la fin pour dire qu'on a fini
    outClock.write( m_syncTime );
}

/**********************************************************************/
/*              Operation Orocos                                      */
/**********************************************************************/

bool MotorSimul::ooLimitCurrent(double ampValue)
{
    return true;
}

void MotorSimul::ooFaulhaberCmd(int cmd, int param)
{
    ArdMotorItf::setOperationMode(ArdMotorItf::OTHER);
}

void MotorSimul::ooSleep(int dt)
{
    sleep(dt);
}

bool MotorSimul::ooSetOperationMode(std::string mode)
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
            LOG(Error) << "Failed to switch to " << mode << " operation mode. Checks syntax or required mode is not available" << endlog();
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

bool MotorSimul::coWaitEnable(double timeout)
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

void MotorSimul::runSpeed()
{
    if( m_power and !m_blockMotor)
    {
        //ne pas changer la commande qui correspond a priori a une conception interne d'un vrai moteur
        outFilteredSpeedCommand.write(m_speedCommand);
        m_speedMeasure = m_speedCommand;
    }
    else if (m_blockMotor)
    {
        outFilteredSpeedCommand.write(m_speedCommand);
        m_speedMeasure = 0;
    }
    else
    {
        outFilteredSpeedCommand.write(0);
        m_speedMeasure = 0;
    }
    outHomingDone.write(false);
}

void MotorSimul::runTorque()
{
    if( m_power )
    {
        //TODO
    }
    outHomingDone.write(false);
}

void MotorSimul::runPosition()
{
    if( m_power )
    {
        //ne pas changer la commande qui correspond a priori a une conception interne d'un vrai moteur
        outFilteredPositionCommand.write(m_positionCommand);
        m_positionMeasure = m_positionCommand;
        //TODO :
        //outFiltereadSpeedCommand.write(??);
        //m_speedMeasure = ?;
    }
    else
    {
        m_speedMeasure = 0;
    }
    outHomingDone.write(false);
}

void MotorSimul::runHoming()
{
    if( m_power )
    {
        m_positionMeasure = 0;
        m_speedMeasure = 0;
        outHomingDone.write(true);
    }
    else
    {
        m_speedMeasure = 0;
    }
}

void MotorSimul::runOther()
{
    outHomingDone.write(false);
}

bool MotorSimul::init()
{
    bool res = true;
    m_power = false;
    return res;
}

void MotorSimul::enableDrive()
{
    m_power = true;
}

void MotorSimul::disableDrive()
{
    m_power = false;
}

bool MotorSimul::reset()
{
    m_power = false;
    return false;
}

bool MotorSimul::getLimitSwitchStatus()
{
    //there is no limit switch currently
    return false;
}

bool MotorSimul::startWatchdog()
{
    //there is no watchdog yet
    return false;
}

bool MotorSimul::stopWatchdog()
{
    //there is no watchdog yet
    return false;
}

bool MotorSimul::isInError()
{
    return false;
}

unsigned int MotorSimul::getError()
{
    return 0;
}
