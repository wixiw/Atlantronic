/*
 * Discovery.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#include "Discovery.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Discovery)

DiscoveryLock::DiscoveryLock(pthread_mutex_t* mutex) :
        m_mutex(mutex)
{

}

DiscoveryLock::~DiscoveryLock()
{
    pthread_mutex_unlock(m_mutex);
}

DiscoveryLock::eLockResult DiscoveryLock::lock()
{
    if (pthread_mutex_lock(m_mutex) != 0)
    {
        return FAILED;
    }
    else
    {
        return SUCCEED;
    }
}

void DiscoveryLock::unlock()
{
    pthread_mutex_unlock(m_mutex);
}
;

Discovery::Discovery(const std::string& name) :
        Stm32TaskContext(name), attrStartPlugged(false),
        attrDynamixelTargetReachedRX24F(RX24_MAX_ID),
        attrDynamixelTargetReachedAX12(AX12_MAX_ID),
        attrDynamixelPositionRX24F(RX24_MAX_ID),
        attrDynamixelPositionAX12(AX12_MAX_ID)
{
    createOrocosInterface();
    createRosInterface();
}

bool Discovery::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    int res = m_robotItf.init("discovery", "/dev/discovery", "/dev/discovery", robotItfCallbackWrapper, this);
    if (res != 0)
    {
        LOG(Error) << "configureHook() : failed to init robot interface with errcode : " << res << endlog();
        return false;
    }

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

void Discovery::cleanupHook()
{
    m_robotItf.destroy();
}

void Discovery::updateHook()
{
    Stm32TaskContext::updateHook();
    Start start;
    StartColor color;

    DiscoveryLock mutex(&m_robotItf.mutex);

    if (mutex.lock() == DiscoveryLock::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    int id = m_robotItf.control_usb_data_count - 1;
    if (id < 0)
    {
        id = 0;
    }

    attrGyrometerAngleEuler = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro_euler);
    attrGyrometerAngleSimpson = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro_simpson);
    attrGyrometerVelocity = m_robotItf.control_usb_data[id].omega_gyro;
    attrGyrometerRawData = m_robotItf.control_usb_data[id].raw_data_gyro;
    attrStartPlugged = m_robotItf.get_gpio(GPIO_IN_GO);
    attrStartColor = m_robotItf.get_gpio(GPIO_COLOR);

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

    attrGyrometerAngleEulerDegree = rad2deg(attrGyrometerAngleEuler);
    attrGyrometerAngleSimpsonDegree = rad2deg(attrGyrometerAngleSimpson);
    attrGyrometerVelocityDegree = rad2deg(attrGyrometerVelocity);

    outGyrometerAngleEuler.write(attrGyrometerAngleEuler);
    outGyrometerAngleEulerDegree.write(attrGyrometerAngleEulerDegree);
    outGyrometerAngleSimpson.write(attrGyrometerAngleSimpson);
    outGyrometerAngleSimpsonDegree.write(attrGyrometerAngleSimpsonDegree);
    outGyrometerVelocity.write(attrGyrometerVelocity);
    outGyrometerVelocityDegree.write(attrGyrometerVelocityDegree);
    outGyrometerRawData.write(attrGyrometerRawData);

    start.go = attrStartPlugged;
    outIoStart.write(start);

    switch (attrStartColor)
    {
        case COLOR_RED:
            color.color = "red";
            break;
        case COLOR_YELLOW:
        default:
            color.color = "yellow";
            break;
    }
    outIoStartColor.write(color);

    //canon
    outRightCannonFingerTargetReached.write(attrDynamixelTargetReachedRX24F[RX24_CANON_SHOOT_RIGHT]);
    outRightCannonStockerTargetReached.write(attrDynamixelTargetReachedAX12[AX12_CANON_STOCK_RIGHT]);
    outRightCannonFingerPosition.write(attrDynamixelPositionRX24F[RX24_CANON_SHOOT_RIGHT]);
    outRightCannonStockerPosition.write(attrDynamixelPositionAX12[AX12_CANON_STOCK_RIGHT]);


}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->updateHook();
}

bool Discovery::ooStartCalibration()
{
    int res = m_robotItf.gyro_calibration(GYRO_CALIBRATION_START);
    if (res < 0)
    {
        LOG(Error) << "Failed to start gyrometer calibration (Command result=" << res << ")." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Starting gyrometer calibration (Command result=" << res << ")." << endlog();
        return true;
    }
}

bool Discovery::ooStopCalibration(double newAngle)
{
    //TODO et le new angle ?
    int res = m_robotItf.gyro_calibration(GYRO_CALIBRATION_STOP);

    if (res < 0)
    {
        LOG(Error) << "Failed to stop gyrometer calibration (Command result=" << res << ")." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Stopping gyrometer calibration (Command result=" << res << ")." << endlog();
        return true;
    }
}

bool Discovery::ooSetPosition(double newAngle)
{
    int res = m_robotItf.gyro_set_position(newAngle);

    if (res < 0)
    {
        LOG(Error) << "Failed to a new position to the gyro (err=" << res << ")." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "New position set for the gyrometer : " << newAngle << " rad." << endlog();
        return true;
    }

}

bool Discovery::ooSetCalibrationValues(double scale, double bias, double dead_zone)
{
    int res = m_robotItf.gyro_set_calibration_values(scale, bias, dead_zone);

    if (res < 0)
    {
        LOG(Error) << "Failed to force new calib (err=" << res << ")." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "New calib forced : scale=" << scale << "    bias=" << bias << "  dead_zone=" << dead_zone
                << endlog();
        return true;
    }
}

bool Discovery::ooReset()
{
    int res = m_robotItf.reboot();

    if (res < 0)
    {
        LOG(Error) << "Failed to reset the stm32 board." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Resetting the stm32 board." << endlog();
        return true;
    }
}

bool Discovery::ooEnableStart()
{
    int res = m_robotItf.go_enable();

    if (res < 0)
    {
        LOG(Error) << "Failed to enable the start in the stm32 board." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Start enabled in the stm32 board." << endlog();
        return true;
    }
}

bool Discovery::ooScanDynamixelRX24F()
{
    int res = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_RX24);

    if (res < 0)
    {
        LOG(Error) << "Failed to scan RX24F." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Scan RX24F success." << endlog();
        return true;
    }
}

bool Discovery::ooScanDynamixelAX12()
{
    int res = m_robotItf.dynamixel_scan(DYNAMIXEL_TYPE_AX12);

    if (res < 0)
    {
        LOG(Error) << "Failed to scan AX12." << endlog();
        return false;
    }
    else
    {
        LOG(Info) << "Scan AX12 success." << endlog();
        return true;
    }
}

bool Discovery::ooDynamixelSetIdRX24F(int oldId, int newId)
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

bool Discovery::ooDynamixelSetIdAX12(int oldId, int newId)
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

bool Discovery::ooDynamixelSetPositionRX24F(int id, double position)
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

bool Discovery::ooDynamixelSetPositionAX12(int id, double position)
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

bool Discovery::ooDynamixelSetTorqueRX24F(int id, int percentage)
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

bool Discovery::ooDynamixelSetTorqueAX12(int id, int percentage)
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

bool Discovery::ooDynamixelSetPrecisionRX24F(int id, double precision)
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

bool Discovery::ooDynamixelSetPrecisionAX12(int id, double precision)
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

bool Discovery::srvStartGyroCalibration(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooStartCalibration();
    return res.success;
}

bool Discovery::srvStopGyroCalibration(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooStopCalibration(req.theta);
    return res.success;
}

bool Discovery::srvSetGyroPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooSetPosition(req.theta);
    return res.success;
}

bool Discovery::srvResetStm32(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooReset();
    return res.success;
}

bool Discovery::srvEnableStart(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooEnableStart();
    return res.success;
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrGyrometerRawData", attrGyrometerRawData);
    addAttribute("attrGyrometerVelocity", attrGyrometerVelocity);
    addAttribute("attrGyrometerVelocityDegree", attrGyrometerVelocityDegree);
    addAttribute("attrGyrometerAngleEuler", attrGyrometerAngleEuler);
    addAttribute("attrGyrometerAngleEulerDegree", attrGyrometerAngleEulerDegree);
    addAttribute("attrGyrometerAngleSimpson", attrGyrometerAngleSimpson);
    addAttribute("attrGyrometerAngleSimpsonDegree", attrGyrometerAngleSimpsonDegree);
    addAttribute("attrStartPlugged", attrStartPlugged);
    addAttribute("attrStartColor", attrStartColor);

//    addAttribute("attrDynamixelTargetReachedRX24F", attrDynamixelTargetReachedRX24F);
//    addAttribute("attrDynamixelTargetReachedAX12", attrDynamixelTargetReachedAX12);
//    addAttribute("attrDynamixelPositionRX24F", attrDynamixelPositionRX24F);
//    addAttribute("attrDynamixelPositionAX12", attrDynamixelPositionAX12);

    addPort("outGyrometerAngleEuler", outGyrometerAngleEuler).doc(
            "Angular position of the gyrometer in rad, integrated with Euler explicit integration scheme");
    addPort("outGyrometerAngleEulerDegree", outGyrometerAngleEulerDegree).doc(
            "Angular position of the gyrometer in degree, integrated with Euler explicit integration scheme");
    addPort("outGyrometerAngleSimpson", outGyrometerAngleSimpson).doc(
            "Angular position of the gyrometer in rad, integrated with Simpson integration scheme");
    addPort("outGyrometerAngleSimpsonDegree", outGyrometerAngleSimpsonDegree).doc(
            "Angular position of the gyrometer in degree, integrated with Simpson integration scheme");
    addPort("outGyrometerVelocity", outGyrometerVelocity).doc("Angular velocity of the gyrometer in rad/sec");
    addPort("outGyrometerVelocityDegree", outGyrometerVelocityDegree).doc(
            "Angular velocity of the gyrometer in degree/sec");
    addPort("outGyrometerRawData", outGyrometerRawData).doc("Angular velocity of the gyrometer in LSB");

    addPort("outIoStart", outIoStart).doc(
            "Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoStartColor", outIoStartColor).doc("Value of the color switch");

    addPort("outRightCannonFingerTargetReached", outRightCannonFingerTargetReached);
    addPort("outRightCannonStockerTargetReached", outRightCannonStockerTargetReached);
    addPort("outRightCannonFingerPosition", outRightCannonFingerPosition);
    addPort("outRightCannonStockerPosition", outRightCannonStockerPosition);

    //Gyrometer
    addOperation("ooStartCalibration", &Discovery::ooStartCalibration, this, OwnThread).doc(
            "Ask the gyrometer to freeze its position and to start the calibration process");
    addOperation("ooStopCalibration", &Discovery::ooStopCalibration, this, OwnThread).doc(
            "Ask the gyrometer to stop the calibration process and to re-publish position datas");
    addOperation("ooSetPosition", &Discovery::ooSetPosition, this, OwnThread).doc(
            "Force a new gyrometer position, in rad");
    addOperation("ooSetCalibrationValues", &Discovery::ooSetCalibrationValues, this, OwnThread).doc(
            "Force new calib params");

    addOperation("ooReset", &Discovery::ooReset, this, OwnThread).doc("Reset the stm32 board.");
    addOperation("ooEnableStart", &Discovery::ooEnableStart, this, OwnThread).doc(
            "Informs the stm32 that the next start withdraw sill be the match begining.");

    //Dynamixels
    addOperation("ooScanDynamixelRX24F", &Discovery::ooScanDynamixelRX24F, this, OwnThread).doc(
            "Scan dynamixels of type RX24F.");
    addOperation("ooScanDynamixelAX12", &Discovery::ooScanDynamixelAX12, this, OwnThread).doc(
            "Scan dynamixels of type RAX12.");
    addOperation("ooDynamixelSetIdRX24F", &Discovery::ooDynamixelSetIdRX24F, this, OwnThread).doc(
            "Set new ID for dynamixel of type RX24F.");
    addOperation("ooDynamixelSetIdAX12", &Discovery::ooDynamixelSetIdAX12, this, OwnThread).doc(
            "Set new ID for dynamixel of type AX12.");
    addOperation("ooDynamixelSetPositionRX24F", &Discovery::ooDynamixelSetPositionRX24F, this, OwnThread).doc(
            "Set target of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetPositionAX12", &Discovery::ooDynamixelSetPositionAX12, this, OwnThread).doc(
            "Set target of dynamixels of type AX12.");
    addOperation("ooDynamixelSetTorqueRX24F", &Discovery::ooDynamixelSetTorqueRX24F, this, OwnThread).doc(
            "Set max torque of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetTorqueAX12", &Discovery::ooDynamixelSetTorqueAX12, this, OwnThread).doc(
            "Set max torque of dynamixels of type AX12.");
    addOperation("ooDynamixelSetPrecisionRX24F", &Discovery::ooDynamixelSetPrecisionRX24F, this, OwnThread).doc(
            "Set precision of dynamixels of type RX24F.");
    addOperation("ooDynamixelSetPrecisionAX12", &Discovery::ooDynamixelSetPrecisionAX12, this, OwnThread).doc(
            "Set precision of dynamixels of type AX12.");

}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvStartGyroCalibration = nh.advertiseService("/Discovery/startGyroCalibration",
            &Discovery::srvStartGyroCalibration, this);
    m_srvStopGyroCalibration = nh.advertiseService("/Discovery/stopGyroCalibration", &Discovery::srvStopGyroCalibration,
            this);
    m_srvSetGyroPosition = nh.advertiseService("/Discovery/setGyroPosition", &Discovery::srvSetGyroPosition, this);
    m_srvResetStm32 = nh.advertiseService("/Discovery/resetStm32", &Discovery::srvResetStm32, this);
    m_srvEnableStart = nh.advertiseService("/Discovery/enableStart", &Discovery::srvEnableStart, this);
}
