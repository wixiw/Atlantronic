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
        Stm32TaskContext(name)
{
    createOrocosInterface();
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

    attrGyrometerAngle = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro);
    mutex.unlock();

    outGyrometerAngle.write(attrGyrometerAngle);
    //TODO
    //outGyrometerRawData.write();
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->updateHook();
}

bool Discovery::ooStartCalibration()
{
    int res = m_robotItf.gyro_calibration(GYRO_CALIBRATION_START);
    if( res <= 0 )
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

bool Discovery::ooStopCalibration()
{
    int res = m_robotItf.gyro_calibration(GYRO_CALIBRATION_STOP);

    if( res <= 0 )
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

void Discovery::ooSetPosition(double newAngle)
{
    DiscoveryLock mutex(&m_robotItf.mutex);

    LOG(Info) << "Setting a new position for the gyrometer : " << newAngle << " rad." << endlog();
    //TODO à implémenter
}

void Discovery::ooReset()
{
    LOG(Info) << "Resetting the stm32 board." << endlog();
    //TODO à implémenter
}

void Discovery::createOrocosInterface()
{
    addAttribute("attrStm32Time", m_robotItf.current_time);
    addAttribute("attrGyrometerAngle", attrGyrometerAngle);


    addPort("outGyrometerAngle", outGyrometerAngle)
        .doc("Angular position of the gyrometer in rad.");
    addPort("outGyrometerRawData", outGyrometerRawData)
        .doc("Unprocessed gyrometer data.");

    addOperation("ooStartCalibration",&Discovery::ooStartCalibration, this, OwnThread)
     .doc("Ask the gyrometer to freeze its position and to start the calibration process");
    addOperation("ooStopCalibration",&Discovery::ooStopCalibration, this, OwnThread)
     .doc("Ask the gyrometer to stop the calibration process and to re-publish position datas");
    addOperation("ooSetPosition",&Discovery::ooSetPosition, this, OwnThread)
     .doc("Force a new gyrometer position, in rad");
    addOperation("ooReset",&Discovery::ooReset, this, OwnThread)
     .doc("Reset the stm32 board.");
}
