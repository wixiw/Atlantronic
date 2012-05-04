/*
 * KFLocalizator.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iomanip>

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
, maxTime4OdoPrediction(0.5)
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
    ss << " [*] maxTime4OdoPrediction: " << maxTime4OdoPrediction << " (sec)" << std::endl;
    ss << " [*] referencedBeacons.size(): " << referencedBeacons.size() << std::endl;
    for(unsigned int i = 0 ; i < referencedBeacons.size() ; i++)
    {
        ss << "       # referencedBeacons[" << i << "]:  ";
        ss << "  x=" << referencedBeacons[i].x() << "(m)";
        ss << "  y=" << referencedBeacons[i].y() << "(m)";
        ss << "  r=" << referencedBeacons[i].r() << "(m)";
        ss << "" << std::endl;
    }
    ss << "****************************" << std::endl;
    ss << iekfParams.getInfo();
    ss << "****************************" << std::endl;
    ss << procParams.getInfo();
    return ss.str();
}

bool KFLocalizator::Params::checkConsistency() const
{
    if(bufferSize < 10)
    {
        Log( NOTICE ) << "KFLocalizator::Params::checkConsistency - inconsistent parameter : bufferSize (" << bufferSize << ") < 10";
        return false;
    }
    if(maxTime4OdoPrediction <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::Params::checkConsistency - inconsistent parameter : maxTime4OdoPrediction (" << maxTime4OdoPrediction << ") should be strictly positive";
        return false;
    }
    if(!iekfParams.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "IEKFParams::Params are not consistent";
        return false;
    }
    if(!procParams.checkConsistency())
    {
        Log( NOTICE ) << "BeaconDetector::Params::checkConsistency" << " - " << "BeaconDetector::Params are not consistent";
        return false;
    }
    return true;
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

bool KFLocalizator::IEKFParams::checkConsistency() const
{
    if(defaultOdoVelTransSigma <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : defaultOdoVelTransSigma (" << defaultOdoVelTransSigma << ") should be strictly positive";
        return false;
    }
    if(defaultOdoVelRotSigma <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : defaultOdoVelRotSigma (" << defaultOdoVelRotSigma << ") should be strictly positive";
        return false;
    }
    if(defaultLaserRangeSigma <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : defaultLaserRangeSigma (" << defaultLaserRangeSigma << ") should be strictly positive";
        return false;
    }
    if(defaultLaserThetaSigma <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : defaultLaserThetaSigma (" << defaultLaserThetaSigma << ") should be strictly positive";
        return false;
    }
    if(iekfMaxIt == 0)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : iekfMaxIt (" << iekfMaxIt << ") should be strictly positive";
        return false;
    }
    if(iekfInnovationMin <= 0.)
    {
        Log( NOTICE ) << "KFLocalizator::IEKFParams::checkConsistency - inconsistent parameter : iekfInnovationMin (" << iekfInnovationMin << ") should be strictly positive";
        return false;
    }
    return true;
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
    if(!p.checkConsistency())
    {
        Log( ERROR ) << "KFLocalizator::setParams - Parameters are ignored because they are inconsistent :\n" << p.getInfo();
        return;
    }
    params.bufferSize = p.bufferSize;
    params.maxTime4OdoPrediction = p.maxTime4OdoPrediction;
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

    kfl::Log( DEBUG ) << "KFLocalizator::initialize - (x=" << pose.x() << ", y=" << pose.y() << " h=" << rad2deg(betweenMinusPi2AndPlusPi2(pose.h())) << " deg)";
    return true;
}

bool KFLocalizator::newOdoVelocity(arp_math::EstimatedTwist2D odoVel)
{
    double startTime = getTime();

    if(circularBuffer.empty())
    {
        Log( ERROR ) << "KFLocalizator::newOdoVelocity - Internal Circular Buffer is empty (maybe you forgot to initialize) => return false";
        return false;
    }

    arp_math::EstimatedPose2D lastEstim = circularBuffer.back().first;

    if( lastEstim.date() >= odoVel.date() )
    {
        Log( ERROR ) << "KFLocalizator::newOdoVelocity - Last estimation date (" << std::setprecision (9) << lastEstim.date() << ") is posterior or same than odo velocity date (" << odoVel.date() << ") => return false";
        return false;
    }

    if( lastEstim.date() + params.maxTime4OdoPrediction < odoVel.date() )
    {
        Log( WARN ) << "KFLocalizator::newOdoVelocity - Time between EstimatedTwist2D date (" << odoVel.date() << ") and last estimation date (" << lastEstim.date() << ") is strangely big and superior than" << params.maxTime4OdoPrediction << "). "
                << "maybe because of an initalization date error or a scheduling fault => return false";
        return false;
    }

    double dt = odoVel.date() - lastEstim.date();

    KFLSysInput input;
    input(0) = odoVel.vx();
    input(1) = odoVel.vy();
    input(2) = odoVel.vh();

    BFLWrapper::PredictParams predictParams;
    predictParams.odoVelXSigma = sqrt(odoVel.cov()(0,0));
    predictParams.odoVelYSigma = sqrt(odoVel.cov()(1,1));
    predictParams.odoVelHSigma = sqrt(odoVel.cov()(2,2));
    bayesian->predict( input, dt, predictParams );

    updateBuffer(odoVel.date(), odoVel);

    double stopTime = getTime();
    Log( DEBUG ) << "KFLocalizator::newOdoVelocity - New odo : dt=" << dt << " (sec)  vx=" << odoVel.vx() << " (m/s)  vy=" << odoVel.vy() << " (m/s)  vh=" << rad2deg(odoVel.vh()) << " (deg/s)  - compute time=" << (stopTime - startTime)*1000. << " (ms)";
    return true;
}

bool KFLocalizator::newScan(lsl::LaserScan scan)
{
    double startTime = getTime();

    Log( DEBUG ) << "KFLocalizator::newScan - *************************************************************";
    Log( DEBUG ) << "KFLocalizator::newScan - enter";

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
    double startBITPTime = arp_math::getTime();

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

    double startInterpTime = arp_math::getTime();

    Eigen::VectorXd tt = scan.getTimeData();
//    Eigen::VectorXi indices = Interpolator::find(tt, ttc);
    Eigen::VectorXi indices = Eigen::VectorXi(0);
    Eigen::VectorXd xx = Interpolator::transInterp(tt, ttFromBuffer, xxFromBuffer, indices);
    Eigen::VectorXd yy = Interpolator::transInterp(tt, ttFromBuffer, yyFromBuffer, indices);
    Eigen::VectorXd hh = Interpolator::rotInterp(tt, ttFromBuffer, hhFromBuffer, indices);
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covs = Interpolator::covInterp(tt, ttFromBuffer, covFromBuffer, indices, 1.e-6, Eigen::Vector3d(1.e-9,1.e-9,1.e-9));
    Eigen::VectorXd vx = Interpolator::transInterp(tt, ttFromBuffer, vxFromBuffer, indices);
    Eigen::VectorXd vy = Interpolator::transInterp(tt, ttFromBuffer, vyFromBuffer, indices);
    Eigen::VectorXd vh = Interpolator::transInterp(tt, ttFromBuffer, vhFromBuffer, indices); // transInterp and not rotInterp because vh is not borned
    Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > odoCovs = Interpolator::covInterp(tt, ttFromBuffer, odoCovFromBuffer, indices, 1.e-9, Eigen::Vector3d(1.e-12,1.e-12,1.e-12));

    popBufferUntilADate(tt(0));

    kfl::Log( INFO ) << "KFLocalizator::newScan - Interpolation Computation Time :" << arp_math::getTime() - startInterpTime;

    if(circularBuffer.empty())
    {
        Log( CRIT ) << "KFLocalizator::newScan - Internal Circular Buffer is empty after \"back to the past\" operation => return false";
        return false;
    }

    kfl::Log( INFO ) << "KFLocalizator::newScan - Back in the Past Computation Time :" << arp_math::getTime() - startBITPTime;
    //***************************************


    double startBPTime = arp_math::getTime();
    beaconDetector.process(scan, tt, xx, yy, hh);
    kfl::Log( INFO ) << "BeaconDetector Computation Time :" << arp_math::getTime() - startBPTime;

    Log( DEBUG ) << "KFLocalizator::newScan - " << beaconDetector.getFoundBeacons().size() << " beacons(s) detected in scan";


    // Reinit in the past
    KFLStateVar initStateVar;
    initStateVar(0) = xx[0];
    initStateVar(1) = yy[0];
    initStateVar(2) = hh[0];

    KFLStateCov initStateCov;
    initStateCov = covs[0];

    BFLWrapper::FilterParams filterParams;
    filterParams.iekfMaxIt = params.iekfParams.iekfMaxIt;
    filterParams.iekfInnovationMin = params.iekfParams.iekfInnovationMin;
    bayesian->init(initStateVar, initStateCov, filterParams);



    KFLStateVar BITPEstimVar = bayesian->getEstimate();
    KFLStateCov BITPEstimCov = bayesian->getCovariance();
    //    Log( DEBUG ) << "KFLocalizator::newScan - estimée in the past [time=" << tt[0] << "]";
    //    Log( DEBUG ) << "  sur x (en m): " << BITPEstimVar(0);
    //    Log( DEBUG ) << "  sur y (en m): " << BITPEstimVar(1);
    //    Log( DEBUG ) << "  en cap (deg) : " << rad2deg( BITPEstimVar(2) );
    //    Log( DEBUG ) << "covariance : " << BITPEstimCov.row(0);
    //    Log( DEBUG ) << "             " << BITPEstimCov.row(1);
    //    Log( DEBUG ) << "             " << BITPEstimCov.row(2);

    //    Log( DEBUG ) << "KFLocalizator::newScan - i=" << 0 << "  - time=" << tt(0) << "  - yy=" << yy(0) << " (m)  - vy=" << vy(0) << " (m/s)  - preOdoY=" << preUpEstim(1);

    // Update
    double startUpdateTime = arp_math::getTime();
    unsigned int nbBeaconSeen = 0;
    for(unsigned int i = 1 ; i < scan.getSize() ; i++)
    {
        lsl::Circle target;
        Eigen::Vector2d meas;
        if( beaconDetector.getBeacon( tt(i), target, meas) )
        {
            KFLStateVar preEstimStateVar = bayesian->getEstimate();
            KFLStateCov preEstimStateCov = bayesian->getCovariance();
            //            Log( DEBUG ) << "KFLocalizator::newScan - pre intermediaire estim [time=" << tt[i] << "]";
            //            Log( DEBUG ) << "  sur x (en m): " << preEstimStateVar(0);
            //            Log( DEBUG ) << "  sur y (en m): " << preEstimStateVar(1);
            //            Log( DEBUG ) << "  en cap (deg) : " << rad2deg( preEstimStateVar(2) );
            //            Log( DEBUG ) << "covariance : " << preEstimStateCov.row(0);
            //            Log( DEBUG ) << "             " << preEstimStateCov.row(1);
            //            Log( DEBUG ) << "             " << preEstimStateCov.row(2);

            BFLWrapper::UpdateParams updateParams;
            updateParams.laserRangeSigma = params.iekfParams.defaultLaserRangeSigma;
            updateParams.laserThetaSigma = params.iekfParams.defaultLaserThetaSigma;
            Log( DEBUG ) << "KFLocalizator::newScan - meas=" << meas.transpose() ;
            Log( DEBUG ) << "KFLocalizator::newScan - target=" << target.getPosition().transpose() ;
            Log( DEBUG ) << "----" ;
            bayesian->update(meas, target.getPosition(), updateParams);
            nbBeaconSeen++;

            KFLStateVar postEstimStateVar = bayesian->getEstimate();
            KFLStateCov postEstimStateCov = bayesian->getCovariance();
            //            Log( DEBUG ) << "KFLocalizator::newScan - post intermediaire estim [time=" << tt[i] << "]";
            //            Log( DEBUG ) << "  sur x (en m): " << postEstimStateVar(0);
            //            Log( DEBUG ) << "  sur y (en m): " << postEstimStateVar(1);
            //            Log( DEBUG ) << "  en cap (deg) : " << rad2deg( postEstimStateVar(2) );
            //            Log( DEBUG ) << "covariance : " << postEstimStateCov.row(0);
            //            Log( DEBUG ) << "             " << postEstimStateCov.row(1);
            //            Log( DEBUG ) << "             " << postEstimStateCov.row(2);
        }

        KFLSysInput input;
        input(0) = vx(i);
        input(1) = vy(i);
        input(2) = vh(i);

        double dt = tt(i) - tt(i-1);

        BFLWrapper::PredictParams predictParams;
        predictParams.odoVelXSigma = sqrt(odoCovs(i)(0,0));
        predictParams.odoVelYSigma = sqrt(odoCovs(i)(1,1));
        predictParams.odoVelHSigma = sqrt(odoCovs(i)(2,2));

        //        KFLStateVar preOdoEstim = bayesian->getEstimate();

        bayesian->predict( input, dt, predictParams );

        //        KFLStateVar postOdoEstim = bayesian->getEstimate();
        //        Log( DEBUG ) << "KFLocalizator::newScan - i=" << i << "  - time=" << tt(i) << "  - yy=" << yy(i) << " (m)  - vy=" << vy(i) << " (m/s)  - preOdoY=" << preOdoEstim(1) << "  - postOdoY=" << postOdoEstim(1);
    }
    Log( DEBUG ) << "KFLocalizator::newScan - " << nbBeaconSeen << " kalman update(s) done";
    double endUpdateTime = arp_math::getTime();
    kfl::Log( INFO ) << "Update Computation Time :" << endUpdateTime - startUpdateTime;

    EstimatedTwist2D T;
    T.vx( vx(tt.size()-1) );
    T.vy( vy(tt.size()-1) );
    T.vh( vh(tt.size()-1) );
    T.cov( odoCovs(tt.size()-1) );
    T.date(tt(tt.size()-1));
    KFLocalizator::updateBuffer(tt(tt.size()-1), T);

    Log( DEBUG ) << "KFLocalizator::newScan - Total computation time =" << (getTime() - startTime)*1000. << " (ms)";
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


std::string KFLocalizator::getPerformanceReport()
{
    return beaconDetector.getPerformanceReport();
}

