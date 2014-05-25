/*
 * DynamixelBus.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/actuators/DynamixelBus.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::DynamixelBus)

DynamixelBus::DynamixelBus(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf)
{
    createOrocosInterface();
}

bool DynamixelBus::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void DynamixelBus::updateHook()
{
    Stm32TaskContext::updateHook();
}

bool DynamixelBus::ooScanRx24F()
{
    int res2 = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_RX24);

    if (res2 < 0)
    {
        LOG(Error) << "Failed to scan Rx24F dynamixels." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Scan Rx24F dynamixel success." << endlog();
        return true;
    }
}

bool DynamixelBus::ooScanAx12()
{
    int res1 = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_AX12);

    if (res1 < 0 )
    {
        LOG(Error) << "Failed to scan Ax12 dynamixels." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Scan Ax12 dynamixel success." << endlog();
        return true;
    }
}

bool DynamixelBus::ooSetId(int family, int oldId, int newId)
{
    int res = m_robotItf.dynamixel_set_id(family, oldId, newId);

    if (res < 0)
    {
        LOG(Error) << "Failed to set new ID with curent id =" << oldId << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel id=" << oldId << " changed to id=" << newId << "." << endlog();
        return true;
    }
}

bool DynamixelBus::ooSetBusBaudRate(int family, int baudrate)
{
    int res = m_robotItf.dynamixel_set_manager_baudrate(family, baudrate);

    if (res < 0)
    {
        LOG(Error) << "Failed to set new bus baudrate for family=" << family << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel Bus baudrate for family " << family << " changed to baudrate=" << baudrate << "." << endlog();
        return true;
    }
}

bool DynamixelBus::ooSetDynamixelBaudRate(int family, int id)
{
    int res = m_robotItf.dynamixel_set_op_baudrate(family, id);

    if (res < 0)
    {
        LOG(Error) << "Failed to set new baudrate for family=" << family << " of id=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel baudrate for family " << family << " of id=" << id << "." << endlog();
        return true;
    }
}

void DynamixelBus::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    addOperation("ooScanRx24F", &DynamixelBus::ooScanRx24F, this, OwnThread).doc(
            "Scan dynamixels present on the bus.");
    addOperation("ooScanAx12", &DynamixelBus::ooScanAx12, this, OwnThread).doc(
                "Scan dynamixels present on the bus.");

    addOperation("ooSetId", &DynamixelBus::ooSetId, this, OwnThread).doc(
            "Set new ID for a dynamixel.");
    addOperation("ooSetBusBaudRate", &DynamixelBus::ooSetBusBaudRate, this, OwnThread).doc(
            "Set a new baudrate for the bus of the family type.");
    addOperation("ooSetDynamixelBaudRate", &DynamixelBus::ooSetDynamixelBaudRate, this, OwnThread).doc(
            "Set a baudrate to 1M for the dynamixel id of the family type type.");
}
