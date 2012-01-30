/*
 * BFLWrapper.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <KFL/BFL/BFLWrapper.hpp>

#include <exceptions/NotImplementedException.hpp>

// BFL includes
#include <filter/extendedkalmanfilter.h>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace kfl;

BFLWrapper::BFLWrapper()
: BayesianWrapper()
{
    ;
}

void BFLWrapper::init(double t, KFLStateVar statevariable, KFLStateCov statecovariance)
{
    throw NotImplementedException();
    return;
}

void BFLWrapper::predict(double dt, KFLSysInput sinp)
{
    throw NotImplementedException();
    return;
}

void BFLWrapper::update(KFLMeasVar mvar, KFLMeasCov mcov, KFLMeasTarget mtar)
{
    throw NotImplementedException();
    return;
}

KFLStateVar BFLWrapper::getEstimate() const
{
    throw NotImplementedException();
    return KFLStateVar();
}

KFLStateCov BFLWrapper::getCovariance() const
{
    throw NotImplementedException();
    return KFLStateCov();
}
