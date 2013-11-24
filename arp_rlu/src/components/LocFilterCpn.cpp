/*
 * LocFilterCpn.cpp
 *
 *  Created on: 9 mai 2012
 *      Author: ard
 */

#include <iomanip>

#include "LocFilterCpn.hpp"
#include <rtt/Component.hpp>
#include "KFL/Logger.hpp"
#include <ros/ros.h>

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::LocFilterCpn )

LocFilterCpn::LocFilterCpn(const std::string& name)
: RluTaskContext(name)
, propCutOffFrequency(5)
, propSamplingFrequency(50)
, initialized(false)
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::kfl::Logger::InitFile("KFL", DEBUG);

    createOrocosInterface();
}

bool LocFilterCpn::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void LocFilterCpn::updateHook()
{
    double w = tan(M_PI * propCutOffFrequency / propSamplingFrequency);
    double a0, a1, a2, b0, b1, b2;
    a0 = w * w;
    a1 = 2. * w * w;
    a2 = w * w;
    b0 = 1. + w * w + sqrt(2.) * w;
    b1 = 2. * (w * w - 1);
    b2 = 1. + w * w - sqrt(2.) * w;

    arp_math::EstimatedPose2D X_n;
    if( RTT::NoData != inPose.read(X_n) )
    {
        if( !initialized )
        {
            Y_nm2 = X_n;
            Y_nm1 = X_n;
            X_nm1 = X_n;
            X_nm2 = X_n;
            initialized = true;
            outPose.write(X_n);
        }
        else
        {
            arp_math::EstimatedPose2D Y_n = X_n;

            Y_n.x( (X_n.x() + X_nm1.x() * a1 + X_nm2.x() * a2 - Y_nm1.x() * b1 - Y_nm2.x() * b2) * a0 / b0 );
            Y_n.y( (X_n.y() + X_nm1.y() * a1 + X_nm2.y() * a2 - Y_nm1.y() * b1 - Y_nm2.y() * b2) * a0 / b0 );
            Y_nm2 = Y_nm1;
            Y_nm1 = Y_n;
            X_nm2 = X_nm1;
            X_nm1 = X_n;
            outPose.write(Y_n);
        }

    }
    return;
}


std::string LocFilterCpn::printParams()
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << "****************************" << std::endl;
    ss << "[*] propCutOffFrequency : " << propCutOffFrequency << " (Hz)" << std::endl;
    ss << "[*] propSamplingFrequency : " << propSamplingFrequency << " (Hz)" << std::endl;
    ss << "****************************" << std::endl;
    return ss.str();
}

bool LocFilterCpn::ooReset()
{
    initialized = false;
    return true;
}

void LocFilterCpn::createOrocosInterface()
{

    addPort("inPose",inPose)
    .doc("Pose with noise");

    addPort("outPose",outPose)
    .doc("Pose without noise");

    addOperation("ooReset",&LocFilterCpn::ooReset, this, OwnThread)
    .doc("");

    addOperation("ooPrintParams",&LocFilterCpn::printParams, this, OwnThread)
    .doc("");

    addProperty( "propCutOffFrequency", propCutOffFrequency);
    addProperty( "propSamplingFrequency", propSamplingFrequency);

}
