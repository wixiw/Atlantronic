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

DiscoveryLock::DiscoveryLock(pthread_mutex_t* mutex):
    m_mutex(mutex)
{

}

DiscoveryLock:: ~DiscoveryLock()
{
    pthread_mutex_unlock(m_mutex);
}

DiscoveryLock::eLockResult DiscoveryLock::lock()
{
    if( pthread_mutex_lock(m_mutex) != 0 )
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
};



Discovery::Discovery(const std::string& name) :
        Stm32TaskContext(name),
        attrStartPlugged(false)
{
    createOrocosInterface();
    createRosInterface();
}

bool Discovery::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    int res = m_robotItf.init("discovery", "/dev/discovery", "/dev/discovery", robotItfCallbackWrapper, this);
    if( res != 0 )
    {
        LOG(Error) << "configureHook() : failed to init robot interface with errcode : " << res << endlog();
        return false;
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

    DiscoveryLock mutex(&m_robotItf.mutex);

    if( mutex.lock() == DiscoveryLock::FAILED)
    {
		LOG(Error) << "updateHook() : mutex.lock()" << endlog();
    }

    int id = m_robotItf.control_usb_data_count - 1;
    if( id < 0)
    {
        id = 0;
    }

    attrGyrometerAngleEuler = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro_euler);
    attrGyrometerAngleSimpson = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro_simpson);
    attrGyrometerVelocity = m_robotItf.control_usb_data[id].omega_gyro;
    attrGyrometerRawData = m_robotItf.control_usb_data[id].raw_data_gyro;
    attrStartPlugged = m_robotItf.get_gpio(GPIO_IN_GO);
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
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->updateHook();
}

bool Discovery::ooStartCalibration()
{
    int res = m_robotItf.gyro_calibration(GYRO_CALIBRATION_START);
    if( res < 0 )
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

    if( res < 0 )
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

    if( res < 0 )
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

        if( res < 0 )
        {
            LOG(Error) << "Failed to force new calib (err=" << res << ")." << endlog();
            return false;
        }
        else
        {
            LOG(Info) << "New calib forced : scale=" << scale << "    bias=" << bias << "  dead_zone=" << dead_zone << endlog();
            return true;
        }
}

bool Discovery::ooReset()
{
    int res = m_robotItf.reboot();

    if( res < 0 )
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

    if( res < 0 )
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
    addAttribute("attrGyrometerRawData",attrGyrometerRawData);
    addAttribute("attrGyrometerVelocity", attrGyrometerVelocity);
    addAttribute("attrGyrometerVelocityDegree", attrGyrometerVelocityDegree);
    addAttribute("attrGyrometerAngleEuler", attrGyrometerAngleEuler);
    addAttribute("attrGyrometerAngleEulerDegree", attrGyrometerAngleEulerDegree);
    addAttribute("attrGyrometerAngleSimpson", attrGyrometerAngleSimpson);
    addAttribute("attrGyrometerAngleSimpsonDegree", attrGyrometerAngleSimpsonDegree);
    addAttribute("attrStartPlugged", attrStartPlugged);

    addPort("outGyrometerAngleEuler", outGyrometerAngleEuler)
        .doc("Angular position of the gyrometer in rad, integrated with Euler explicit integration scheme");
    addPort("outGyrometerAngleEulerDegree", outGyrometerAngleEulerDegree)
        .doc("Angular position of the gyrometer in degree, integrated with Euler explicit integration scheme");
    addPort("outGyrometerAngleSimpson", outGyrometerAngleSimpson)
        .doc("Angular position of the gyrometer in rad, integrated with Simpson integration scheme");
    addPort("outGyrometerAngleSimpsonDegree", outGyrometerAngleSimpsonDegree)
        .doc("Angular position of the gyrometer in degree, integrated with Simpson integration scheme");
    addPort("outGyrometerVelocity", outGyrometerVelocity)
        .doc("Angular velocity of the gyrometer in rad/sec");
    addPort("outGyrometerVelocityDegree", outGyrometerVelocityDegree)
        .doc("Angular velocity of the gyrometer in degree/sec");
    addPort("outGyrometerRawData", outGyrometerRawData)
           .doc("Angular velocity of the gyrometer in LSB");

    addPort("outIoStart",outIoStart)
        .doc("Value of the start. GO is true when it is not in, go is false when the start is in");
    addPort("outIoStartColor",outIoStartColor)
        .doc("Value of the color switch");

    addOperation("ooStartCalibration",&Discovery::ooStartCalibration, this, OwnThread)
     .doc("Ask the gyrometer to freeze its position and to start the calibration process");
    addOperation("ooStopCalibration",&Discovery::ooStopCalibration, this, OwnThread)
     .doc("Ask the gyrometer to stop the calibration process and to re-publish position datas");
    addOperation("ooSetPosition",&Discovery::ooSetPosition, this, OwnThread)
     .doc("Force a new gyrometer position, in rad");
    addOperation("ooSetCalibrationValues",&Discovery::ooSetCalibrationValues, this, OwnThread)
     .doc("Force new calib params");
    addOperation("ooReset",&Discovery::ooReset, this, OwnThread)
     .doc("Reset the stm32 board.");
    addOperation("ooEnableStart",&Discovery::ooEnableStart, this, OwnThread)
     .doc("Informs the stm32 that the next start withdraw sill be the match begining.");
}

void Discovery::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvStartGyroCalibration   = nh.advertiseService("/Discovery/startGyroCalibration",    &Discovery::srvStartGyroCalibration, this);
    m_srvStopGyroCalibration    = nh.advertiseService("/Discovery/stopGyroCalibration",   &Discovery::srvStopGyroCalibration, this);
    m_srvSetGyroPosition        = nh.advertiseService("/Discovery/setGyroPosition",         &Discovery::srvSetGyroPosition, this);
    m_srvResetStm32             = nh.advertiseService("/Discovery/resetStm32", &Discovery::srvResetStm32, this);
    m_srvEnableStart            = nh.advertiseService("/Discovery/enableStart", &Discovery::srvEnableStart, this);
}
