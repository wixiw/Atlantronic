/*
 * Gyrometer.cpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#include "components/sensors/Gyrometer.hpp"
#include "components/discovery/DiscoveryMutex.hpp"
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
        attrRawData(0),
        attrVelocity(0),
        attrVelocityDegree(0),
        attrAngleEuler(0),
        attrAngleEulerDegree(0),
        attrAngleSimpson(0),
        attrAngleSimpsonDegree(0),
        propCalibratedBiais(0.0),
        propCalibratedScale(0.0),
        propDeadZone(0.0)
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

    attrAngleEuler = betweenMinusPiAndPlusPi(last_data->pos_theta_gyro_euler);
    attrAngleSimpson = betweenMinusPiAndPlusPi(last_data->pos_theta_gyro_simpson);
    attrVelocity = last_data->omega_gyro;
    attrRawData = last_data->raw_data_gyro;

    mutex.unlock();

    attrAngleEulerDegree = rad2deg(attrAngleEuler);
    attrAngleSimpsonDegree = rad2deg(attrAngleSimpson);
    attrVelocityDegree = rad2deg(attrVelocity);

    outAngleEuler.write(attrAngleEuler);
    outAngleEulerDegree.write(attrAngleEulerDegree);
    outAngleSimpson.write(attrAngleSimpson);
    outAngleSimpsonDegree.write(attrAngleSimpsonDegree);
    outVelocity.write(attrVelocity);
    outVelocityDegree.write(attrVelocityDegree);
    outRawData.write(attrRawData);
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



bool Gyrometer::srvStartCalibration(EmptyWithSuccess::Request& req, EmptyWithSuccess::Response& res)
{
    res.success = ooStartCalibration();
    return res.success;
}

bool Gyrometer::srvStopCalibration(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooStopCalibration(req.theta);
    return res.success;
}

bool Gyrometer::srvSetPosition(SetGyroPosition::Request& req, SetGyroPosition::Response& res)
{
    res.success = ooSetPosition(req.theta);
    return res.success;
}

void Gyrometer::createOrocosInterface()
{
    addAttribute("attrStm32Time",           m_robotItf.current_time);
    addAttribute("attrRawData",             attrRawData);
    addAttribute("attrVelocity",            attrVelocity);
    addAttribute("attrVelocityDegree",      attrVelocityDegree);
    addAttribute("attrAngleEuler",          attrAngleEuler);
    addAttribute("attrAngleEulerDegree",    attrAngleEulerDegree);
    addAttribute("attrAngleSimpson",        attrAngleSimpson);
    addAttribute("attrAngleSimpsonDegree",  attrAngleSimpsonDegree);

    addProperty("propCalibratedBiais",      propCalibratedBiais);
    addProperty("propCalibratedScale",      propCalibratedScale);
    addProperty("propDeadZone",             propDeadZone);

    addPort("outAngleEuler", outAngleEuler).doc(
            "Angular position of the gyrometer in rad, integrated with Euler explicit integration scheme");
    addPort("outAngleEulerDegree", outAngleEulerDegree).doc(
            "Angular position of the gyrometer in degree, integrated with Euler explicit integration scheme");
    addPort("outAngleSimpson", outAngleSimpson).doc(
            "Angular position of the gyrometer in rad, integrated with Simpson integration scheme");
    addPort("outAngleSimpsonDegree", outAngleSimpsonDegree).doc(
            "Angular position of the gyrometer in degree, integrated with Simpson integration scheme");
    addPort("outVelocity", outVelocity).doc("Angular velocity of the gyrometer in rad/sec");
    addPort("outVelocityDegree", outVelocityDegree).doc(
            "Angular velocity of the gyrometer in degree/sec");
    addPort("outRawData", outRawData).doc("Angular velocity of the gyrometer in LSB");

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
    nh.advertiseService("/Gyrometer/startGyroCalibration",
            &Gyrometer::srvStartCalibration, this);
    nh.advertiseService("/Gyrometer/stopGyroCalibration", &Gyrometer::srvStopCalibration,
            this);
    nh.advertiseService("/Gyrometer/setGyroPosition", &Gyrometer::srvSetPosition, this);
}
