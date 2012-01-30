/*
 * BFLWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BFLWRAPPER_HPP_
#define _ARP_RLU_KFL_BFLWRAPPER_HPP_

#include <math/math.hpp>
#include <KFL/BayesianWrapper.hpp>

namespace arp_rlu
{

namespace kfl
{

class BFLWrapper : BayesianWrapper
{
    public:
    BFLWrapper();

    void init(double t, KFLStateVar, KFLStateCov);
    void predict(double dt, KFLSysInput );
    void update(KFLMeasVar, KFLMeasCov, KFLMeasTarget);
    KFLStateVar getEstimate() const;
    KFLStateCov getCovariance() const;


    protected:

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BFLWRAPPER_HPP_ */
