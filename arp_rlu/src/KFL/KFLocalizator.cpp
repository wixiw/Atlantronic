/*
 * KFLocalizator.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "KFLocalizator.hpp"

#include "KFL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

// BFL include
#include <KFL/BFL/BFLWrapper.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;


KFLocalizator::Params::Params()
: bufferSize(100)
, referencedBeacons()
, iekfParams(KFLocalizator::IEKFParams())
, procParams(BeaconDetector::Params())
{
}

std::string KFLocalizator::Params::getInfo() const
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
    ss << iekfParams.getInfo();
    ss << "****************************" << std::endl;
    ss << procParams.getInfo();
    return ss.str();
}

KFLocalizator::IEKFParams::IEKFParams()
: defaultOdoVelTransSigma(0.001)
, defaultOdoVelRotSigma(0.01)
, defaultLaserRangeSigma(0.005)
, defaultLaserThetaSigma(0.05)
, iekfMaxIt(10)
, iekfInnovationMin(0.00015)
{
}

std::string KFLocalizator::IEKFParams::getInfo() const
{
    std::stringstream ss;
    ss << "KFLocalizator IEKFParams :" << std::endl;
    ss << " [*] defaultOdoVelTransSigma : " << defaultOdoVelTransSigma << std::endl;
    ss << " [*] defaultOdoVelRotSigma   : " << defaultOdoVelRotSigma << std::endl;
    ss << " [*] defaultLaserRangeSigma  : " << defaultLaserRangeSigma << std::endl;
    ss << " [*] defaultLaserThetaSigma  : " << defaultLaserThetaSigma << std::endl;
    ss << " [*] iekfMaxIt               : " << iekfMaxIt << std::endl;
    ss << " [*] iekfInnovationMin       : " << iekfInnovationMin << std::endl;
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
    setParams(p.iekfParams);
    setParams(p.procParams);
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

bool KFLocalizator::initialize(const arp_math::EstimatedPose2D & pose)
{
    circularBuffer.rset_capacity(params.bufferSize);
    circularBuffer.clear();

    double initDate = pose.date();

    KFLStateVar initStateVar;
    initStateVar(0) = pose.x();
    initStateVar(1) = pose.y();
    initStateVar(2) = pose.h();

    KFLStateCov initStateCov;
    initStateCov = pose.cov();

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = params.iekfParams.iekfMaxIt;
    filterParams.iekfInnovationMin = params.iekfParams.iekfInnovationMin;
    bayesian->init(initStateVar, initStateCov, filterParams);

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

    if( lastEstim.date() >= odoVel.date() )
    {
        Log( ERROR ) << "KFLocalizator::newOdoVelocity - Last estimation date (" << lastEstim.date() << ") is posterior or same than odo velocity date (" << odoVel.date() << ") => return false";
        return false;
    }

    double dt = odoVel.date() - lastEstim.date();

    KFLSysInput input;
    input(0) = odoVel.vx();
    input(1) = odoVel.vy();
    input(2) = odoVel.vh();


    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = odoVel.cov()(0,0);
    predictParams.odoVelYSigma = odoVel.cov()(1,1);
    predictParams.odoVelHSigma = odoVel.cov()(2,2);
    bayesian->predict( input, dt, predictParams );

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
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covFromBuffer(circularBuffer.size());
    Eigen::VectorXd vxFromBuffer(circularBuffer.size());
    Eigen::VectorXd vyFromBuffer(circularBuffer.size());
    Eigen::VectorXd vhFromBuffer(circularBuffer.size());
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > odoCovFromBuffer(circularBuffer.size());
    for( unsigned int i = 0 ; i < circularBuffer.size() ; i++ )
    {
        ttFromBuffer(i)     = circularBuffer[i].first.date();
        xxFromBuffer(i)     = circularBuffer[i].first.x();
        yyFromBuffer(i)     = circularBuffer[i].first.y();
        hhFromBuffer(i)     = circularBuffer[i].first.h();
        covFromBuffer(i)    = circularBuffer[i].first.cov();
        vxFromBuffer(i)     = circularBuffer[i].second.vx();
        vyFromBuffer(i)     = circularBuffer[i].second.vy();
        vhFromBuffer(i)     = circularBuffer[i].second.vh();
        odoCovFromBuffer(i) = circularBuffer[i].second.cov();
    }

    Eigen::VectorXd tt = scan.getTimeData();
    Eigen::VectorXd xx = Interpolator::transInterp(tt, ttFromBuffer, xxFromBuffer);
    Eigen::VectorXd yy = Interpolator::transInterp(tt, ttFromBuffer, yyFromBuffer);
    Eigen::VectorXd hh = Interpolator::rotInterp(tt, ttFromBuffer, hhFromBuffer);
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covs = Interpolator::covInterp(tt, ttFromBuffer, covFromBuffer, 1.e-6, Eigen::Vector3d(1.e-9,1.e-9,1.e-9));
    Eigen::VectorXd vx = Interpolator::transInterp(tt, ttFromBuffer, vxFromBuffer);
    Eigen::VectorXd vy = Interpolator::transInterp(tt, ttFromBuffer, vyFromBuffer);
    Eigen::VectorXd vh = Interpolator::transInterp(tt, ttFromBuffer, vhFromBuffer); // transInterp and not rotInterp because vh is not borned
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > odoCovs = Interpolator::covInterp(tt, ttFromBuffer, odoCovFromBuffer, 1.e-6, Eigen::Vector3d(1.e-9,1.e-9,1.e-9));

    popBufferUntilADate(tt(0));

    if(circularBuffer.empty())
    {
        Log( CRIT ) << "KFLocalizator::newScan - Internal Circular Buffer is empty after \"back to the past\" operation => return false";
        return false;
    }
    //***************************************


    beaconDetector.process(scan, tt, xx, yy, hh);

    Log( DEBUG ) << "KFLocalizator::newScan - " << beaconDetector.getFoundBeacons().size() << " beacons(s) detected in scan";


    // Update
    unsigned int nbBeaconSeen = 0;
    for(unsigned int i = 1 ; i < scan.getSize() ; i++)
    {
        lsl::Circle target;
        Eigen::Vector2d meas;
        if( beaconDetector.getBeacon( tt(i), target, meas) )
        {
            BFLWrapper::UpdateParams updateParams;
            updateParams.laserRangeSigma = params.iekfParams.defaultLaserRangeSigma;
            updateParams.laserThetaSigma = params.iekfParams.defaultLaserThetaSigma;
//            Log( DEBUG ) << "KFLocalizator::newScan - meas=" << meas.transpose() ;
//            Log( DEBUG ) << "KFLocalizator::newScan - target=" << target.getPosition().transpose() ;
            bayesian->update(meas, target.getPosition(), updateParams);
            nbBeaconSeen++;
        }

        KFLSysInput input;
        input(0) = vx(i);
        input(1) = vy(i);
        input(2) = vh(i);

        double dt = tt(i) - tt(i-1);

        BFLWrapper::PredictParams predictParams;
        predictParams.odoVelXSigma = odoCovs(i)(0,0);
        predictParams.odoVelYSigma = odoCovs(i)(1,1);
        predictParams.odoVelHSigma = odoCovs(i)(2,2);

        bayesian->predict( input, dt, predictParams );
    }
    Log( DEBUG ) << "KFLocalizator::newScan - " << nbBeaconSeen << " kalman update(s) done";

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

