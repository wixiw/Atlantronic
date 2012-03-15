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
: bufferSize(100)
, referencedBeacons()
, initParams(KFLocalizator::InitParams())
, iekfParams(KFLocalizator::IEKFParams())
, procParams(BeaconDetector::Params())
{
    referencedBeacons.push_back( lsl::Circle( 1.5, 0., 0.04 ) );
    referencedBeacons.push_back( lsl::Circle(-1.5, 1., 0.04 ) );
    referencedBeacons.push_back( lsl::Circle(-1.5,-1., 0.04 ) );
}

std::string KFLocalizator::Params::getInfo()
{
    std::stringstream ss;
    ss << "KFLocalizator Params :" << std::endl;
    ss << " [*] bufferSize: " << bufferSize << std::endl;
    ss << " [*] referencedBeacons.size(): " << referencedBeacons.size() << std::endl;
    for(unsigned int i = 0 ; i < referencedBeacons.size() ; i++)
    {
        ss << "       # referencedBeacons[" << i << "]:  ";
        ss << "  x=" << referencedBeacons[i].x();
        ss << "  y=" << referencedBeacons[i].y();
        ss << "  r=" << referencedBeacons[i].r();
        ss << "" << std::endl;
    }
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
, circularBuffer(boost::circular_buffer< std::pair< arp_math::EstimatedPose2D, arp_math::EstimatedTwist2D > >(params.bufferSize))
{
    bayesian = new BFLWrapper();
    setParams(params);
}

KFLocalizator::~KFLocalizator()
{
    if(bayesian)
        delete bayesian;
}

void KFLocalizator::setParams(KFLocalizator::Params p)
{
    params.bufferSize = p.bufferSize;
    params.referencedBeacons = p.referencedBeacons;
    beaconDetector.setReferencedBeacons(params.referencedBeacons);
    setParams(params.initParams);
    setParams(params.iekfParams);
    setParams(params.procParams);
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
    beaconDetector.setParams(params.procParams);
}

bool KFLocalizator::initialize()
{
    circularBuffer.rset_capacity(params.bufferSize);
    circularBuffer.clear();

    double initDate = params.initParams.initialPose.date();

    KFLStateVar initStateVar;
    initStateVar(0) = params.initParams.initialPose.x();
    initStateVar(1) = params.initParams.initialPose.y();
    initStateVar(2) = params.initParams.initialPose.h();

    KFLStateCov initStateCov;
    initStateCov = params.initParams.initialPose.cov();

    bayesian->init(initStateVar, initStateCov);

    updateBuffer(initDate);
    return true;
}

bool KFLocalizator::newOdoVelocity(arp_math::EstimatedTwist2D odoVel)
{
    if(circularBuffer.empty())
    {
        Log( ERROR ) << "KFLocalizator::newOdoVelocity - Internal Circular Buffer is empty (maybe you forgot to initialize) => return false";
        return false;
    }

    arp_math::EstimatedPose2D lastEstim = circularBuffer.back().first;

    if( lastEstim.date() > odoVel.date() )
    {
        Log( ERROR ) << "KFLocalizator::newOdoVelocity - Last estimation date is posterior than odo velocity date (?!?) => return false";
        return false;
    }

    double dt = odoVel.date() - lastEstim.date();

    KFLSysInput input;
    input(0) = dt * odoVel.vx();
    input(1) = dt * odoVel.vy();
    input(2) = dt * odoVel.vh();

    bayesian->predict( input );

    updateBuffer(odoVel.date(), odoVel);
    return true;
}

bool KFLocalizator::newScan(lsl::LaserScan scan)
{
    if(circularBuffer.empty())
    {
        Log( ERROR ) << "KFLocalizator::newScan - Internal Circular Buffer is empty (maybe you forgot to initialize it) => return false";
        return false;
    }

    if(params.referencedBeacons.empty())
    {
        Log( ERROR ) << "KFLocalizator::newScan - No referenced Beacon (maybe you forgot to reference them) => return false";
        return false;
    }

    if(scan.getSize() == 0)
    {
        Log( WARN ) << "KFLocalizator::newScan - Scan is empty => return false";
        return false;
    }

    if(circularBuffer.front().first.date() >= scan.getTimeData()(0))
    {
        Log( WARN ) << "KFLocalizator::newScan - Not enough history in circular buffer => return false";
        return false;
    }

    //***************************************
    // back to the past
    Eigen::VectorXd ttFromBuffer(circularBuffer.size());
    Eigen::VectorXd xxFromBuffer(circularBuffer.size());
    Eigen::VectorXd yyFromBuffer(circularBuffer.size());
    Eigen::VectorXd hhFromBuffer(circularBuffer.size());
    Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > covFromBuffer(circularBuffer.size());
    Eigen::VectorXd vxFromBuffer(circularBuffer.size());
    Eigen::VectorXd vyFromBuffer(circularBuffer.size());
    Eigen::VectorXd vhFromBuffer(circularBuffer.size());
    for( unsigned int i = 0 ; i < circularBuffer.size() ; i++ )
    {
        ttFromBuffer(i)  = circularBuffer[i].first.date();
        xxFromBuffer(i)  = circularBuffer[i].first.x();
        yyFromBuffer(i)  = circularBuffer[i].first.y();
        hhFromBuffer(i)  = circularBuffer[i].first.h();
        covFromBuffer(i) = circularBuffer[i].first.cov();
        vxFromBuffer(i)  = circularBuffer[i].second.vx();
        vyFromBuffer(i)  = circularBuffer[i].second.vy();
        vhFromBuffer(i)  = circularBuffer[i].second.vh();
    }

    Eigen::VectorXd tt = scan.getTimeData();
    Eigen::VectorXd xx = Interpolator::transInterp(tt, ttFromBuffer, xxFromBuffer);
    Eigen::VectorXd yy = Interpolator::transInterp(tt, ttFromBuffer, yyFromBuffer);
    Eigen::VectorXd hh = Interpolator::rotInterp(tt, ttFromBuffer, hhFromBuffer);
    Eigen::Array< Eigen::Matrix3d, Dynamic, 1 > covs = Interpolator::covInterp(tt, ttFromBuffer, covFromBuffer, 1.e-6, Eigen::Vector3d(1.e-9,1.e-9,1.e-9));
    Eigen::VectorXd vx = Interpolator::transInterp(tt, ttFromBuffer, vxFromBuffer);
    Eigen::VectorXd vy = Interpolator::transInterp(tt, ttFromBuffer, vyFromBuffer);
    Eigen::VectorXd vh = Interpolator::transInterp(tt, ttFromBuffer, vhFromBuffer); // transInterp and not rotInterp because vh is not borned

    popBufferUntilADate(tt(0));

    if(circularBuffer.empty())
    {
        Log( CRIT ) << "KFLocalizator::newScan - Internal Circular Buffer is empty after \"back to the past\" operation => return false";
        return false;
    }
    //***************************************


    beaconDetector.process(scan, tt, xx, yy, hh);

    for(unsigned int i = 0 ; i < scan.getSize() ; i++)
    {
        lsl::Circle target;
        Eigen::Vector2d meas;
        if( beaconDetector.getBeacon( tt(i), target, meas) )
        {
            KFLMeasVar m;
            KFLMeasCov c;
            KFLMeasTarget t;
            // TODO : ici définir m, c et t
            bayesian->update(m, c, t);
        }
        // TODO : ici faire la prediction avec un dt adéquat
    }

    KFLocalizator::updateBuffer(tt(tt.size()-1), EstimatedTwist2D(vx(tt.size()-1), vy(tt.size()-1), vh(tt.size()-1)));

    return false;
}

arp_math::EstimatedPose2D KFLocalizator::getLastEstimatedPose2D()
{
    if(circularBuffer.empty())
    {
        Log( ERROR ) << "KFLocalizator::getLastEstimatedPose2D - Internal Circular Buffer is empty (maybe you forgot to initialize) => return arp_math::EstimatedPose2D()";
        return arp_math::EstimatedPose2D();
    }
    return circularBuffer.back().first;
}

arp_math::EstimatedTwist2D KFLocalizator::getLastEstimatedTwist2D()
{
    if(circularBuffer.empty())
    {
        Log( ERROR ) << "KFLocalizator::getLastEstimatedPose2D - Internal Circular Buffer is empty (maybe you forgot to initialize) => return arp_math::EstimatedTwist2D()";
        return arp_math::EstimatedTwist2D();
    }
    return circularBuffer.back().second;
}

void KFLocalizator::updateBuffer(const double date, const EstimatedTwist2D & tw)
{
    KFLStateVar estimStateVar = bayesian->getEstimate();
    KFLStateCov estimStateCov = bayesian->getCovariance();

    arp_math::EstimatedPose2D estimPose;
    estimPose.date( date );
    estimPose.x( estimStateVar(0) );
    estimPose.y( estimStateVar(1) );
    estimPose.h( estimStateVar(2) );
    estimPose.cov( estimStateCov );

    circularBuffer.push_back( std::make_pair(estimPose, tw) );
}

void KFLocalizator::popBufferUntilADate(const double date)
{
    while(circularBuffer.empty())
    {
        if( circularBuffer.back().first.date() >= date )
        {
            circularBuffer.pop_back();
        }
        else
        {
            return;
        }
    }
}

