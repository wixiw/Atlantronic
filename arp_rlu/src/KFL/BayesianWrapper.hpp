/*
 * BayesianWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_
#define _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_

#include <math/core>

#include <KFL/KFLVariables.hpp>

namespace arp_rlu
{

namespace kfl
{

class BayesianWrapper
{
    public:
        virtual void init(double t, KFLStateVar , KFLStateCov ) = 0;
        virtual void predict(double dt, KFLSysInput ) = 0;
        virtual void update(KFLMeasVar, KFLMeasCov, KFLMeasTarget) = 0;
        virtual KFLStateVar getEstimate() const = 0;
        virtual KFLStateCov getCovariance() const = 0;
};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_ */
