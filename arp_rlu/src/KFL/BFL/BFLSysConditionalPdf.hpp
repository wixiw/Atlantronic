/*
 * BFLSysConditionalPdf.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BFLSYSCONDITIONALPDF_HPP_
#define _ARP_RLU_KFL_BFLSYSCONDITIONALPDF_HPP_

#include <math/math.hpp>

#include "KFL/KFLVariables.hpp"

// BFL includes
#include <wrappers/matrix/matrix_wrapper.h>
#include <pdf/linearanalyticconditionalgaussian.h>


namespace arp_rlu
{

namespace kfl
{

class BFLSysConditionalPdf : public BFL::LinearAnalyticConditionalGaussian
{
    public:
        BFLSysConditionalPdf( const std::vector< MatrixWrapper::Matrix > &ratio, const BFL::Gaussian &additiveNoise);
        ~BFLSysConditionalPdf();

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BFLSYSCONDITIONALPDF_HPP_ */
