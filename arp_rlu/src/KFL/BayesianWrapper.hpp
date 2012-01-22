/*
 * BayesianWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_
#define _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_

#include <math/math.hpp>
#include "KFLVariables.hpp"

namespace arp_rlu
{

namespace kfl
{

class BayesianWrapper
{
    public:
        virtual void init() = 0;
};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_ */
