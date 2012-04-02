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

ORO_LIST_COMPONENT_TYPE( arp_hml::MotorSimul )

MotorSimul::MotorSimul(const std::string& name):
    HmlTaskContext(name),
    ArdMotorItf()
{
    addAttribute("attrCommandedSpeed",m_speedCommand);
    addAttribute("attrPeriod",attrPeriod);
    addAttribute("attrBlockingDelay",attrBlockingDelay);
    addAttribute("attrIncrementalOdometer",attrIncrementalOdometer);

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

    addPort("inSpeedCmd",inSpeedCmd)
            .doc("Command to be used in speed mode. It must be provided in rad/s on the reductor's output");
    addPort("inPositionCmd",inPositionCmd)
            .doc("Command to be used in position mode. It must be provided in rad on the reductor's output. It is not available yet.");
    addPort("inTorqueCmd",inTorqueCmd)
            .doc("Command to be used in torque mode. This mode is not available yes");

    addPort("outFilteredSpeedCommand",outFilteredSpeedCommand)
            .doc("");
    addPort("outFilteredPositionCommand",outFilteredPositionCommand)
            .doc("");

    addPort("outMeasuredPosition",outMeasuredPosition)
        .doc("Provides the measured position of the encoder from CAN. It is converted in rad on the reductor's output's axe.");
    addPort("outMeasuredPositionTime",outMeasuredPositionTime)
        .doc("");
    addPort("outMeasuredTorque",outMeasuredTorque)
        .doc("Provides the torque measured from CAN. Not available yet");
    addPort("outComputedSpeed",outComputedSpeed)
        .doc(" Provides a computed speed from the encoder position. In rad/s on the reductor's output's axe.");
    addPort("outDriveEnable",outDriveEnable)
        .doc("Is true when the drive is ready to be operated (axe blocked). If it is false, the axe is free of any mouvement");
    addPort("outCurrentOperationMode",outCurrentOperationMode)
        .doc("Provides the current mode of operation of the motor (speed,position,torque,homing,other=faulhaber)");
    addPort("outMaxTorqueTimeout",outMaxTorqueTimeout)
        .doc(" Is true when the propMaximalTorque has been reached for more propBlockingTorqueTimeout");
    addPort("outConnected",outConnected)
        .doc("Always true");

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
    outConnected.write(true);
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
}

void MotorSimul::runTorque()
{
}

void MotorSimul::runPosition()
{
}

void MotorSimul::runHoming()
{
}

void MotorSimul::runOther()
{
}

bool MotorSimul::init()
{
    bool res = true;
    return res;
}

void MotorSimul::enableDrive()
{
}

void MotorSimul::disableDrive()
{
}

bool MotorSimul::reset()
{
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
