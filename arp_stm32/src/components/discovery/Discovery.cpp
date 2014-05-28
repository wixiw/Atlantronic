/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace arp_core;
using namespace arp_stm32;
using namespace std;
using namespace RTT;

#define LOG(level) RTT::log(level)<<"["<<getName()<<"] "

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

Discovery::Discovery(const std::string& name) :
        MotionScheduler(name, "arp_stm32"),
        m_robotItf(DiscoveryMutex::robotItf),
        propDeviceName("/dev/discovery"),
        propStm32BootDelay(2.0)
{
    createOrocosInterface();
    createRosInterface();
}

Discovery::~Discovery()
{
    m_robotItf.destroy();
}

bool Discovery::configureHook()
{
    if (!MotionScheduler::configureHook())
        return false;

    int res = m_robotItf.init("discovery", propDeviceName.c_str(), propDeviceName.c_str(), NULL, robotItfCallbackWrapper, this);
    if (res != 0)
    {
        LOG(Error) << "configureHook() : failed to init robot_itf with error code : " << res << endlog();
        return false;
    }

    waitStm32BootDone(propStm32BootDelay);

    return true;
}

bool Discovery::startHook()
{
    bool res = ooReset();
    res &= MotionScheduler::startHook();
    return res;
}

void Discovery::cleanupHook()
{
    m_robotItf.destroy();
    MotionScheduler::cleanupHook();
}



void Discovery::updateHook()
{
    MotionScheduler::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    attrDebugGpio                       = getRawGpioData();
    attrBatteryVoltage                  = getBatteryVoltage();
    attrIsPowerOn                       = isPowerOn();
    attrIsHeartbeatLost                 = isHeartBeatLost();
    attrIsPowerDownDueToEmergencyStop   = isEmergencyStopActive();
    attrIsPowerDownDueToEndMatch        = isPowerShutdownAtEndOfMatch();
    attrIsConnected                     = m_robotItf.isConnected();
    mutex.unlock();

    PowerStatusMsg msg;
    msg.voltage = attrBatteryVoltage;
    msg.isPowerOn = attrIsPowerOn;
    msg.isEmergencyStopActive = attrIsPowerDownDueToEmergencyStop;
    outPowerStatus.write(msg);

    std_msgs::Bool powerRequestMsg;
    if( RTT::NewData == inPowerRequest.read(powerRequestMsg) )
    {
        ooPowerOn(powerRequestMsg.data);
    }

    arp_math::EstimatedPose2D pos;
    inPose.read(pos);
    m_robotItf.set_position(VectPlan(pos.x(), pos.y(), pos.angle()));

    //TODO workaround en attendant implem
//    std_msgs::Empty heartbeatUpdateMsg;
//    if( RTT::NewData == inHeartbeat.read(heartbeatUpdateMsg) )
//    {
//        sendHeartBeat();
//    }
    sendHeartBeat();
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    //Discovery* discovery = (Discovery*) arg;
    //discovery->updateHook();
}

bool Discovery::isPowerOn()
{
    return getRawPowerData() == POWER_ON;
}

bool Discovery::isEmergencyStopActive()
{
    return getRawPowerData() & POWER_OFF_AU;
}

bool Discovery::isUnderVoltageErrorActive()
{
    return getRawPowerData() & POWER_OFF_UNDERVOLTAGE;
}

bool Discovery::isPowerAllowedByStragety()
{
    return getRawPowerData() & POWER_OFF;
}

bool Discovery::isPowerShutdownAtEndOfMatch()
{
    return getRawPowerData() & POWER_OFF_END_MATCH;
}

bool Discovery::isHeartBeatLost()
{
    return getRawPowerData() & POWER_OFF_HEARTBEAT;
}

int Discovery::getRawPowerData()
{
    return m_robotItf.last_control_usb_data.power_state;
}

double Discovery::getBatteryVoltage()
{
    return m_robotItf.last_control_usb_data.vBat;
}

int Discovery::getRawGpioData()
{
    return m_robotItf.last_control_usb_data.gpio;
}

void Discovery::sendHeartBeat()
{
    int errorCode = m_robotItf.heartbeat_update();
    if (errorCode < 0)
    {
        LOG(Error) << "Failed to send to heartbeat." << endlog();
    }
}

bool Discovery::ooReset()
{
    int res = m_robotItf.reboot();

    if (res < 0)
    {
        LOG(Error) << "ooReset() : failed to reboot with error code : " << res << endlog();
        return false;
    }

    return waitStm32BootDone(propStm32BootDelay);
}

bool Discovery::waitStm32BootDone(double timeout)
{
    //polling on the comunication status to wait the stm32 to boot
    double chrono = 0.0;
    whileTimeout( m_robotItf.isConnected() == false , timeout , 0.050 )
    IfWhileTimeoutExpired( timeout )
    {
         LOG(Error) << "Failed to reset STM32"  << endlog();
         return false;
    }

    return true;
}

bool Discovery::ooPowerOn(bool on)
{
    int res = m_robotItf.power_off(!on);

    if (res < 0)
    {
        LOG(Error) << "sendPower() : failed to require power with error code : " << res << endlog();
        return false;
    }

    return true;
}

bool Discovery::srvResetStm32(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooReset();
    return res.success;
}

bool Discovery::srvPowerOn(SetPowerSrv::Request& req, SetPowerSrv::Response& res)
{
    res.success = ooPowerOn(req.powerOn);
    return res.success;
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time",                       m_robotItf.current_time);
    addAttribute("attrDebugGpio",                       attrDebugGpio);
    addAttribute("attrDebugPower",                      attrDebugPower);
    addAttribute("attrIsPowerOn",                       attrIsPowerOn);
    addAttribute("attrBatteryVoltage",                  attrBatteryVoltage);
    addAttribute("attrIsHeartbeatLost",                 attrIsHeartbeatLost);
    addAttribute("attrIsPowerDownDueToEndMatch",        attrIsPowerDownDueToEndMatch);
    addAttribute("attrIsPowerDownDueToEmergencyStop",   attrIsPowerDownDueToEmergencyStop);
    addAttribute("attrIsConnected",                     attrIsConnected);


    addProperty("propDeviceName",                       propDeviceName);
    addProperty("propStm32BootDelay",                   propStm32BootDelay);

    addPort("outPowerStatus",                           outPowerStatus);
    addEventPort("inHeartbeat",                         inHeartbeat);
    addEventPort("inPowerRequest",                      inPowerRequest);
    addPort("inPose",                                   inPose);

    addOperation("ooReset", &Discovery::ooReset, this, OwnThread).doc("Reset the stm32 board.");
    addOperation("ooPowerOn", &Discovery::ooPowerOn, this, OwnThread).doc("Set the power on the stm32 board.");
}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
    nh.advertiseService("/Ubiquity/resetStm32", &Discovery::srvResetStm32, this);
    nh.advertiseService("/Ubiquity/powerOn", &Discovery::srvPowerOn, this);
}
