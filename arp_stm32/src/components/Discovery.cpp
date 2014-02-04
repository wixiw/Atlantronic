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

Discovery::Discovery(const std::string& name) :
        Stm32TaskContext(name)
{
    createOrocosInterface();
}

bool Discovery::configureHook()
{
    if (!Stm32TaskContext::configureHook())
        return false;

    if( m_robotItf.init("discovery", "/dev/discovery0", "/dev/discovery0", robotItfCallbackWrapper, this) )
    {
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

    int res = pthread_mutex_lock(&(m_robotItf.mutex));
    if(res)
    {
        LOG(Error) << "pthread_mutex_lock : " << res << endlog();
    }
    else
    {
        int id = m_robotItf.control_usb_data_count - 1;
        if( id < 0)
        {
            id = 0;
        }

        attrGyrometerAngle = betweenMinusPiAndPlusPi(m_robotItf.control_usb_data[id].pos_theta_gyro);
    }
    pthread_mutex_unlock(&(m_robotItf.mutex));

    outGyrometerAngle.write(attrGyrometerAngle);
    //TODO
    //outGyrometerRawData.write();
}

void Discovery::robotItfCallbackWrapper(void* arg)
{
    Discovery* discovery = (Discovery*) arg;
    discovery->updateHook();
}

void Discovery::ooStartCalibration()
{
    LOG(Info) << "Starting gyrometer calibration." << endlog();
    //TODO à implémenter
}

void Discovery::ooStopCalibration()
{
    LOG(Info) << "Stopping gyrometer calibration." << endlog();
    //TODO à implémenter
}

void Discovery::ooSetPosition(double newAngle)
{
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
