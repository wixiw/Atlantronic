/*
 * KFLocalizator.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "KFLocalizator.hpp"

#include "KFL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace kfl;
using namespace arp_core::log;

#include <KFL/BFL/BFLWrapper.hpp>

KFLocalizator::Params::Params()
{
}

std::string KFLocalizator::Params::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator Params :" << std::endl;
    ss << " [*] bufferSize: " << bufferSize << std::endl;
    ss << "****************************" << std::endl;
    ss << initParams.getInfo();
    ss << "****************************" << std::endl;
    ss << iekfParams.getInfo();
    ss << "****************************" << std::endl;
    ss << procParams.getInfo();
    return ss.str();
}

KFLocalizator::InitParams::InitParams()
{
}

std::string KFLocalizator::InitParams::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator InitParams :" << std::endl;
    ss << " [*] x    : " << initialPose.x() << " (m)" << std::endl;
    ss << " [*] y    : " << initialPose.y() << " (m)" << std::endl;
    ss << " [*] h    : " << rad2deg(initialPose.h()) << " (deg)" << std::endl;
    ss << " [*] date : " << initialPose.date() << " (s)" << std::endl;
    ss << " [*] cov  : " << initialPose.cov() << std::endl;
    return ss.str();
}

KFLocalizator::IEKFParams::IEKFParams()
{
}

std::string KFLocalizator::IEKFParams::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator IEKFParams :" << std::endl;
    ss << " [*] defaultOdoVelTransSigma : " << defaultOdoVelTransSigma << std::endl;
    ss << " [*] defaultOdoVelRotSigma   : " << defaultOdoVelRotSigma << std::endl;
    ss << " [*] defaultLaserRangeSigma  : " << defaultLaserRangeSigma << std::endl;
    ss << " [*] defaultLaserThetaSigma  : " << defaultLaserThetaSigma << std::endl;
    ss << " [*] iekfMaxIt               : " << iekfMaxIt << std::endl;
    return ss.str();
}

KFLocalizator::KFLocalizator()
 : params()
 , beaconDetector()
 , bayesian(NULL)
{
     ;
}

KFLocalizator::~KFLocalizator()
{
    if(bayesian)
        delete bayesian;
}

void KFLocalizator::setParams(KFLocalizator::Params p)
{
    params = p;
}

void KFLocalizator::setParams(KFLocalizator::InitParams p)
{
    params.initParams = p;
}

void KFLocalizator::setParams(KFLocalizator::IEKFParams p)
{
    params.iekfParams = p;
}

void KFLocalizator::setParams(BeaconDetector::Params p)
{
    params.procParams = p;
}

bool KFLocalizator::initialize()
{
    throw NotImplementedException();
    return false;
}

bool KFLocalizator::newOdoVelocity(arp_math::EstimatedTwist2D odoVel)
{
    throw NotImplementedException();
    return false;
}

bool KFLocalizator::newScan(lsl::LaserScan scan)
{
    throw NotImplementedException();
    return false;
}

arp_math::EstimatedPose2D KFLocalizator::getLastEstimatedPose2D()
{
    throw NotImplementedException();
    return arp_math::EstimatedPose2D();
}

arp_math::EstimatedTwist2D KFLocalizator::getLastEstimatedTwist2D()
{
    throw NotImplementedException();
    return arp_math::EstimatedTwist2D();
}

