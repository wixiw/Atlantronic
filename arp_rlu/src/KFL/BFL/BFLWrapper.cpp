/*
 * BFLWrapper.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <KFL/BFL/BFLWrapper.hpp>

#include "KFL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

// BFL includes

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;


BFLWrapper::FilterParams::FilterParams()
: BayesianWrapper::FilterParams()
, defaultOdoVelTransSigma(0.001)
, defaultOdoVelRotSigma(0.01)
, defaultLaserRangeSigma(0.005)
, defaultLaserThetaSigma(0.05)
, iekfMaxIt(10)
, iekfInnovationMin(0.00015)
{
}

std::string BFLWrapper::FilterParams::getInfo() const
{
    std::stringstream ss;
    ss << "BFLWrapper FilterParams :" << std::endl;
    ss << " [*] defaultOdoVelTransSigma : " << defaultOdoVelTransSigma << std::endl;
    ss << " [*] defaultOdoVelRotSigma   : " << defaultOdoVelRotSigma << std::endl;
    ss << " [*] defaultLaserRangeSigma  : " << defaultLaserRangeSigma << std::endl;
    ss << " [*] defaultLaserThetaSigma  : " << defaultLaserThetaSigma << std::endl;
    ss << " [*] iekfMaxIt               : " << iekfMaxIt << std::endl;
    return ss.str();
}


BFLWrapper::BFLWrapper()
: sysPdf(NULL)
, sysModel(NULL)
, measPdf(NULL)
, measModel(NULL)
, innovationCheck(NULL)
, filter(NULL)
{
}

void BFLWrapper::init(const KFLStateVar & statevariable, const KFLStateCov & statecovariance, BayesianWrapper::FilterParams & p)
{
    BFLWrapper::FilterParams params;
    try {
        params = dynamic_cast< BFLWrapper::FilterParams & >(p);
    }
    catch(...)
    {
        Log( ERROR ) << "BFLWrapper::init - BayesianWrapper::FilterParams argument is not a BFLWrapper::FilterParams => return";
        return;
    }

    clear();

    // create gaussian to model sys uncertainty
    MatrixWrapper::ColumnVector sysNoiseMu(3);
    sysNoiseMu(1) = 0.;
    sysNoiseMu(2) = 0.;
    sysNoiseMu(3) = 0.;

    MatrixWrapper::SymmetricMatrix sysNoiseCov(3);
    sysNoiseCov(1,1) = 1.;
    sysNoiseCov(1,2) = 0.0;
    sysNoiseCov(1,3) = 0.0;
    sysNoiseCov(2,1) = 0.0;
    sysNoiseCov(2,2) = 1.;
    sysNoiseCov(2,3) = 0.0;
    sysNoiseCov(3,1) = 0.0;
    sysNoiseCov(3,2) = 0.0;
    sysNoiseCov(3,3) = 1.;

    BFL::Gaussian systemUncertainty(sysNoiseMu, sysNoiseCov);

    MatrixWrapper::Matrix A(3,3);
    A(1,1) = 1.0;
    A(1,2) = 0.0;
    A(1,3) = 0.0;
    A(2,1) = 0.0;
    A(2,2) = 1.0;
    A(2,3) = 0.0;
    A(3,1) = 0.0;
    A(3,2) = 0.0;
    A(3,3) = 1.0;
    MatrixWrapper::Matrix B(3,3);
    B(1,1) = 0.;
    B(1,2) = 0.;
    B(1,3) = 0.;
    B(2,1) = 0.;
    B(2,2) = 0.;
    B(2,3) = 0.;
    B(3,1) = 0.;
    B(3,2) = 0.;
    B(3,3) = 0.;

    std::vector<MatrixWrapper::Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;

    sysPdf = new BFLSysConditionalPdf(AB, systemUncertainty);
    sysModel = new BFL::LinearAnalyticSystemModelGaussianUncertainty(sysPdf);


    // create gaussian to model meas uncertainty
    MatrixWrapper::ColumnVector measNoiseMu(2);
    measNoiseMu(1) = 0.;
    measNoiseMu(2) = 0.;

    MatrixWrapper::SymmetricMatrix measNoiseCov(2);
    measNoiseCov(1,1) = params.defaultLaserRangeSigma;
    measNoiseCov(1,2) = 0.0;
    measNoiseCov(2,1) = 0.0;
    measNoiseCov(2,2) = params.defaultLaserThetaSigma;

    BFL::Gaussian measUncertainty(measNoiseMu, measNoiseCov);

    measPdf = new BFLMeasConditionalPdf(measUncertainty);
    measModel = new BFL::AnalyticMeasurementModelGaussianUncertainty(measPdf);


    // create gaussian to model prior
    MatrixWrapper::ColumnVector priorNoiseMu(3);
    priorNoiseMu(1) = statevariable(0);
    priorNoiseMu(2) = statevariable(1);
    priorNoiseMu(3) = statevariable(2);

    MatrixWrapper::SymmetricMatrix priorNoiseCov(3);
    priorNoiseCov(1,1) = statecovariance(0,0);
    priorNoiseCov(1,2) = statecovariance(0,1);
    priorNoiseCov(1,3) = statecovariance(0,2);
    priorNoiseCov(2,1) = statecovariance(1,0);
    priorNoiseCov(2,2) = statecovariance(1,1);
    priorNoiseCov(2,3) = statecovariance(1,2);
    priorNoiseCov(3,1) = statecovariance(2,0);
    priorNoiseCov(3,2) = statecovariance(2,1);
    priorNoiseCov(3,3) = statecovariance(2,2);

    BFL::Gaussian prior(measNoiseMu, measNoiseCov);


    innovationCheck = new BFL::InnovationCheck( params.iekfInnovationMin );
    filter = new BFL::IteratedExtendedKalmanFilter(&prior, params.iekfMaxIt, innovationCheck);

    return;
}

void BFLWrapper::predict( const KFLSysInput & input , double dt )
{
    if(sysPdf==NULL || sysModel==NULL || measPdf==NULL || measModel==NULL || innovationCheck==NULL || filter==NULL)
    {
        Log( ERROR ) << "BFLWrapper::predict - BFLWrapper has not been initialized => return";
        return;
    }

    throw NotImplementedException();
    return;
}

void BFLWrapper::update( const KFLMeasVar & mvar, const KFLMeasCov & mcov, const KFLMeasTarget & mtar)
{
    if(sysPdf==NULL || sysModel==NULL || measPdf==NULL || measModel==NULL || innovationCheck==NULL || filter==NULL)
    {
        Log( ERROR ) << "BFLWrapper::predict - BFLWrapper has not been initialized => return";
        return;
    }

    throw NotImplementedException();
    return;
}

KFLStateVar BFLWrapper::getEstimate() const
{
    if(sysPdf==NULL || sysModel==NULL || measPdf==NULL || measModel==NULL || innovationCheck==NULL || filter==NULL)
    {
        Log( ERROR ) << "BFLWrapper::predict - BFLWrapper has not been initialized => return KFLStateVar::Zero";
        return KFLStateVar::Zero();
    }

    KFLStateVar ret;
    BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter->PostGet();
    MatrixWrapper::ColumnVector val = posterior->ExpectedValueGet();
    for(unsigned int i = 0 ; i < val.rows() ; i ++)
    {
        ret(i) = val(i+1);
    }
    return ret;
}

KFLStateCov BFLWrapper::getCovariance() const
{
    if(sysPdf==NULL || sysModel==NULL || measPdf==NULL || measModel==NULL || innovationCheck==NULL || filter==NULL)
    {
        Log( ERROR ) << "BFLWrapper::predict - BFLWrapper has not been initialized => return KFLStateCov::Identity()";
        return KFLStateCov::Identity();
    }

    KFLStateCov ret;
    BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter->PostGet();
    MatrixWrapper::SymmetricMatrix cov = posterior->CovarianceGet();
    for(unsigned int i = 0 ; i < cov.rows() ; i ++)
    {
        for(unsigned int j = 0 ; j < cov.columns() ; j ++)
        {
            ret(i,j) = cov(i+1, j+1);
        }
    }
    return ret;
}

void BFLWrapper::clear()
{
    if(sysModel)
    {
        delete sysModel;
        sysModel = NULL;
    }
    if(sysPdf)
    {
        delete sysPdf;
        sysPdf = NULL;
    }
    if(measModel)
    {
        delete measModel;
        measModel = NULL;
    }
    if(measPdf)
    {
        delete measPdf;
        measPdf = NULL;
    }
    if(filter)
    {
        delete filter;
        filter = NULL;
    }
    if(innovationCheck)
    {
        delete innovationCheck;
        innovationCheck = NULL;
    }
    return;
}
