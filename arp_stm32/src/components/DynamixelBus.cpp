/*
 * DynamixelBus.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "DynamixelBus.hpp"
#include "DiscoveryMutex.hpp"
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
        m_robotItf(DiscoveryMutex::robotItf),
        attrDynamixelTargetReachedRX24F(RX24_MAX_ID),
        attrDynamixelTargetReachedAX12(AX12_MAX_ID),
        attrDynamixelPositionRX24F(RX24_MAX_ID),
        attrDynamixelPositionAX12(AX12_MAX_ID)
{
    createOrocosInterface();
    createRosInterface();
}

bool DynamixelBus::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    //TODO checker les retours
    //Limitation des couples par défaut par sécurité e; attendant mieux.
    for (int id = 2; id < RX24_MAX_ID; id++)
    {
        ooDynamixelSetTorqueRX24F(id, 20);
        ooDynamixelSetPrecisionRX24F(id,0.17);
    }
    for (int id = 2; id < AX12_MAX_ID; id++)
    {
        ooDynamixelSetTorqueAX12(id, 20);
        ooDynamixelSetPrecisionAX12(id,0.17);
    }

    return true;
}

void DynamixelBus::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    control_usb_data* last_data = &(m_robotItf.last_control_usb_data);

    for (int id = 2; id < RX24_MAX_ID; id++)
    {
        attrDynamixelTargetReachedRX24F[id]  = ( (m_robotItf.rx24[id].flags & DYNAMIXEL_FLAG_TARGET_REACHED ) != 0 );
        attrDynamixelPositionRX24F[id] = m_robotItf.rx24[id].pos;
    }
    for (int id = 2; id < AX12_MAX_ID; id++)
    {
        attrDynamixelTargetReachedAX12[id] = ( (m_robotItf.ax12[id].flags & DYNAMIXEL_FLAG_TARGET_REACHED ) != 0 );
        attrDynamixelPositionAX12[id] = m_robotItf.ax12[id].pos;
    }

    mutex.unlock();

    //canon
    outRightCannonFingerTargetReached.write(attrDynamixelTargetReachedRX24F[RX24_CANON_SHOOT_RIGHT]);
    outRightCannonStockerTargetReached.write(attrDynamixelTargetReachedAX12[AX12_CANON_STOCK_RIGHT]);
    outRightCannonFingerPosition.write(attrDynamixelPositionRX24F[RX24_CANON_SHOOT_RIGHT]);
    outRightCannonStockerPosition.write(attrDynamixelPositionAX12[AX12_CANON_STOCK_RIGHT]);
}

bool DynamixelBus::ooScanDynamixels()
{
    int res1 = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_RX24);
    int res2 = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_AX12);

    if (res1 < 0 || res2 < 0)
    {
        LOG(Error) << "Failed to scan dynamixels." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Scan dynamixel success." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetIdRX24F(int oldId, int newId)
{
    int res = m_robotItf.dynamixel_set_id(DYNAMIXEL_TYPE_RX24, oldId, newId);

    if (res < 0)
    {
        LOG(Error) << "Failed to set new ID to RX24F with curent id =" << oldId << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel RX24F id=" << oldId << " changed to id=" << newId << "." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetIdAX12(int oldId, int newId)
{
    int res = m_robotItf.dynamixel_set_id(DYNAMIXEL_TYPE_AX12, oldId, newId);

    if (res < 0)
    {
        LOG(Error) << "Failed to set new ID to AX12 with curent id =" << oldId << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel AX12 id=" << oldId << " changed to id=" << newId << "." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetPositionRX24F(int id, double position)
{
    int res = m_robotItf.dynamixel_set_goal_position(DYNAMIXEL_TYPE_RX24, id, position);

    if (res < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel RX24F with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Debug) << "Dynamixel RX24F position set successfully." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetPositionAX12(int id, double position)
{
    int res = m_robotItf.dynamixel_set_goal_position(DYNAMIXEL_TYPE_AX12, id, position);

    if (res < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel AX12 with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Debug) << "Dynamixel AX12 position set successfully." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetTorqueRX24F(int id, int percentage)
{
    int res = m_robotItf.dynamixel_set_max_torque(DYNAMIXEL_TYPE_RX24, id, percentage);

    if (res < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel RX24F with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel RX24F position set successfully." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetTorqueAX12(int id, int percentage)
{
    int res = m_robotItf.dynamixel_set_max_torque(DYNAMIXEL_TYPE_AX12, id, percentage);

    if (res < 0)
    {
        LOG(Error) << "Failed to set position to dynamixel AX12 with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel AX12 position set successfully." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetPrecisionRX24F(int id, double precision)
{
    int res = m_robotItf.dynamixel_set_target_reached_threshold(DYNAMIXEL_TYPE_RX24, id, precision);

    if (res < 0)
    {
        LOG(Error) << "Failed to set precision to dynamixel RX24F with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel RX24F precision set successfully." << endlog();
        return true;
    }
}

bool DynamixelBus::ooDynamixelSetPrecisionAX12(int id, double precision)
{
    int res = m_robotItf.dynamixel_set_target_reached_threshold(DYNAMIXEL_TYPE_AX12, id, precision);

    if (res < 0)
    {
        LOG(Error) << "Failed to set precision to dynamixel AX12 with ID=" << id << "." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Dynamixel AX12 precision set successfully." << endlog();
        return true;
    }
}

void DynamixelBus::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);

    //TODO pourquoi ça bug ça encore ? t(-è__àç_ç_ de typekit
//    addAttribute("attrDynamixelTargetReachedRX24F", attrDynamixelTargetReachedRX24F);
//    addAttribute("attrDynamixelTargetReachedAX12", attrDynamixelTargetReachedAX12);
//    addAttribute("attrDynamixelPositionRX24F", attrDynamixelPositionRX24F);
//    addAttribute("attrDynamixelPositionAX12", attrDynamixelPositionAX12);

    addPort("outRightCannonFingerTargetReached", outRightCannonFingerTargetReached);
    addPort("outRightCannonStockerTargetReached", outRightCannonStockerTargetReached);
    addPort("outRightCannonFingerPosition", outRightCannonFingerPosition);
    addPort("outRightCannonStockerPosition", outRightCannonStockerPosition);

    addOperation("ooScanDynamixels", &DynamixelBus::ooScanDynamixels, this, OwnThread).doc(
            "Scan dynamixels.");
    addOperation("ooDynamixelSetIdRX24F", &DynamixelBus::ooDynamixelSetIdRX24F, this, OwnThread).doc(
            "Set new ID for dynamixel of type RX24F.");
    addOperation("ooDynamixelSetIdAX12", &DynamixelBus::ooDynamixelSetIdAX12, this, OwnThread).doc(
            "Set new ID for dynamixel of type AX12.");
    addOperation("ooDynamixelSetPositionRX24F", &DynamixelBus::ooDynamixelSetPositionRX24F, this, OwnThread).doc(
            "Set target of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetPositionAX12", &DynamixelBus::ooDynamixelSetPositionAX12, this, OwnThread).doc(
            "Set target of dynamixels of type AX12.");
    addOperation("ooDynamixelSetTorqueRX24F", &DynamixelBus::ooDynamixelSetTorqueRX24F, this, OwnThread).doc(
            "Set max torque of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetTorqueAX12", &DynamixelBus::ooDynamixelSetTorqueAX12, this, OwnThread).doc(
            "Set max torque of dynamixels of type AX12.");
    addOperation("ooDynamixelSetPrecisionRX24F", &DynamixelBus::ooDynamixelSetPrecisionRX24F, this, OwnThread).doc(
            "Set precision of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetPrecisionAX12", &DynamixelBus::ooDynamixelSetPrecisionAX12, this, OwnThread).doc(
            "Set precision of dynamixels of type AX12.");

}

void DynamixelBus::createRosInterface()
{
    ros::NodeHandle nh;
}
