/*
 * Gyrometer.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "Gyrometer.hpp"
#include "DiscoveryMutex.hpp"
#include <rtt/Component.hpp>
#include <math/math.hpp>

using namespace arp_math;
using namespace arp_stm32;
using namespace arp_core;
using namespace std;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_stm32::Gyrometer)

Gyrometer::Gyrometer(const std::string& name) :
        Stm32TaskContext(name),
        m_robotItf(DiscoveryMutex::robotItf),
        attrGyrometerRawData(0),
        attrGyrometerVelocity(0),
        attrGyrometerVelocityDegree(0),
        attrGyrometerAngleEuler(0),
        attrGyrometerAngleEulerDegree(0),
        attrGyrometerAngleSimpson(0),
        attrGyrometerAngleSimpsonDegree(0)
{
    createOrocosInterface();
    createRosInterface();
}

bool Gyrometer::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    return true;
}

void Gyrometer::updateHook()
{
    Stm32TaskContext::updateHook();

    DiscoveryMutex mutex;

    if (mutex.lock() == DiscoveryMutex::FAILED)
    {
        LOG(Error) << "updateHook() : mutex.lock()" << endlog();
        return;
    }

    control_usb_data* last_data = &(m_robotItf.last_control_usb_data);

    attrGyrometerAngleEuler = betweenMinusPiAndPlusPi(last_data->pos_theta_gyro_euler);
    attrGyrometerAngleSimpson = betweenMinusPiAndPlusPi(last_data->pos_theta_gyro_simpson);
    attrGyrometerVelocity = last_data->omega_gyro;
    attrGyrometerRawData = last_data->raw_data_gyro;

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
}

bool Gyrometer::ooStartCalibration()
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

bool Gyrometer::ooStopCalibration(double newAngle)
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

bool Gyrometer::ooSetPosition(double newAngle)
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

bool Gyrometer::ooSetCalibrationValues(double scale, double bias, double dead_zone)
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



bool Gyrometer::srvStartGyroCalibration(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooStartCalibration();
    return res.success;
}

bool Gyrometer::srvStopGyroCalibration(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooStopCalibration(req.theta);
    return res.success;
}

bool Gyrometer::srvSetGyroPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooSetPosition(req.theta);
    return res.success;
}

void Gyrometer::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrGyrometerRawData", attrGyrometerRawData);
    addAttribute("attrGyrometerVelocity", attrGyrometerVelocity);
    addAttribute("attrGyrometerVelocityDegree", attrGyrometerVelocityDegree);
    addAttribute("attrGyrometerAngleEuler", attrGyrometerAngleEuler);
    addAttribute("attrGyrometerAngleEulerDegree", attrGyrometerAngleEulerDegree);
    addAttribute("attrGyrometerAngleSimpson", attrGyrometerAngleSimpson);
    addAttribute("attrGyrometerAngleSimpsonDegree", attrGyrometerAngleSimpsonDegree);

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

    //Gyrometer
    addOperation("ooStartCalibration", &Gyrometer::ooStartCalibration, this, OwnThread).doc(
            "Ask the gyrometer to freeze its position and to start the calibration process");
    addOperation("ooStopCalibration", &Gyrometer::ooStopCalibration, this, OwnThread).doc(
            "Ask the gyrometer to stop the calibration process and to re-publish position datas");
    addOperation("ooSetPosition", &Gyrometer::ooSetPosition, this, OwnThread).doc(
            "Force a new gyrometer position, in rad");
    addOperation("ooSetCalibrationValues", &Gyrometer::ooSetCalibrationValues, this, OwnThread).doc(
            "Force new calib params");
}

void Gyrometer::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvStartGyroCalibration = nh.advertiseService("/Gyrometer/startGyroCalibration",
            &Gyrometer::srvStartGyroCalibration, this);
    m_srvStopGyroCalibration = nh.advertiseService("/Gyrometer/stopGyroCalibration", &Gyrometer::srvStopGyroCalibration,
            this);
    m_srvSetGyroPosition = nh.advertiseService("/Gyrometer/setGyroPosition", &Gyrometer::srvSetGyroPosition, this);
}
