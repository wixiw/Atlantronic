/*
 * HmlCmdMockup.cpp
 *
 *  Created on: 03 fev. 2012
 *      Author: wla
 */

#include "HmlCmdMockup.hpp"
#include <rtt/Component.hpp>
#include <rtt/scripting/StateMachine.hpp>

using namespace arp_hml;
using namespace std;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::HmlCmdMockup )

HmlCmdMockup::HmlCmdMockup(const std::string& name):
    HmlTaskContext(name)
{
    addPort("outLeftDrivingSpeedCmd",outLeftDrivingSpeedCmd);
    addPort("outRightDrivingSpeedCmd",outRightDrivingSpeedCmd);
    addPort("outRearDrivingSpeedCmd",outRearDrivingSpeedCmd);
    addPort("outLeftSteeringSpeedCmd",outLeftSteeringSpeedCmd);
    addPort("outRightSteeringSpeedCmd",outRightSteeringSpeedCmd);
    addPort("outRearSteeringSpeedCmd",outRearSteeringSpeedCmd);

    addPort("outLeftDrivingPositionCmd",outLeftDrivingPositionCmd);
    addPort("outRightDrivingPositionCmd",outRightDrivingPositionCmd);
    addPort("outRearDrivingPositionCmd",outRearDrivingPositionCmd);
    addPort("outLeftSteeringPositionCmd",outLeftSteeringPositionCmd);
    addPort("outRightSteeringPositionCmd",outRightSteeringPositionCmd);
    addPort("outRearSteeringPositionCmd",outRearSteeringPositionCmd);

    addPort("outLeftDrivingTorqueCmd",outLeftDrivingTorqueCmd);
    addPort("outRightDrivingTorqueCmd",outRightDrivingTorqueCmd);
    addPort("outRearDrivingTorqueCmd",outRearDrivingTorqueCmd);
    addPort("outLeftSteeringTorqueCmd",outLeftSteeringTorqueCmd);
    addPort("outRightSteeringTorqueCmd",outRightSteeringTorqueCmd);
    addPort("outRearSteeringTorqueCmd",outRearSteeringTorqueCmd);

    addPort("outBit01",outBit01);
    addPort("outBit02",outBit02);
    addPort("outBit03",outBit03);
    addPort("outBit04",outBit04);
    addPort("outBit05",outBit05);
    addPort("outBit06",outBit06);
    addPort("outBit07",outBit07);
    addPort("outBit08",outBit08);
}

bool HmlCmdMockup::loadStateMachines()
{
    string path = attrProjectRootPath + "/" + attrStateMachinePath + "/";

    if( HmlTaskContext::loadStateMachines() == false )
    {
        goto failed;
    }

    if( scripting->loadStateMachines(path + "IoOutSelfTest.osd") == false )
    {
        goto failed;
    }
    if( scripting->loadStateMachines(path + "MotorSelfTest.osd") == false )
    {
        goto failed;
    }

    goto succeed;

    failed:
        return false;
    succeed:
        return true;
}

bool HmlCmdMockup::startHook()
{
    std::vector< std::string > smList = scripting->getStateMachineList();
    std::vector< std::string >::iterator smName;
    scripting::StateMachinePtr sm;

    if( HmlTaskContext::startHook() == false )
    {
        goto failed;
    }

    for( smName = smList.begin() ; smName != smList.end() ; smName++ )
    {
        if( scripting->activateStateMachine(*smName) == false )
        {
            LOG(Error) << "Failed to activate sm " << *smName << "." << endlog();
            goto failed;
        }
        if( scripting->startStateMachine(*smName) == false )
        {
            LOG(Error) << "Failed to start sm " << *smName << "." << endlog();
            goto failed;
        }
    }

    goto succeed;

    failed:
        return false;
    succeed:
        return true;
}

void HmlCmdMockup::stopHook()
{
    std::vector< std::string > smList = scripting->getStateMachineList();
    std::vector< std::string >::iterator smName;
    scripting::StateMachinePtr sm;

    for( smName = smList.begin() ; smName != smList.end() ; smName++ )
    {
        if( scripting->stopStateMachine(*smName) == false )
        {
            LOG(Error) << "Failed to stop sm " << *smName << "." << endlog();
        }
        if( scripting->deactivateStateMachine(*smName) == false )
        {
            LOG(Error) << "Failed to deactivate sm " << *smName << "." << endlog();
        }
    }


    HmlTaskContext::stopHook();
}

void HmlCmdMockup::cleanupHook()
{
    HmlTaskContext::cleanupHook();
}
