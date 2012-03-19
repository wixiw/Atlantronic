/*
 * BFLMeasConditionalPdf.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BFLMEASCONDITIONALPDF_HPP_
#define _ARP_RLU_KFL_BFLMEASCONDITIONALPDF_HPP_

#include <math/math.hpp>

// BFL includes
#include <wrappers/matrix/matrix_wrapper.h>
#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace arp_rlu
{

namespace kfl
{

class BFLMeasConditionalPdf : public BFL::AnalyticConditionalGaussianAdditiveNoise
{
    public:
        BFLMeasConditionalPdf(const BFL::Gaussian& additiveNoise);
        virtual ~BFLMeasConditionalPdf();

        virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
        virtual MatrixWrapper::Matrix          dfGet(unsigned int i) const;


    protected:

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BFLMEASCONDITIONALPDF_HPP_ */
