/*
 * BFLMeasConditionalPdf.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BFLMeasConditionalPdf.hpp"

#include "KFL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;

BFLMeasConditionalPdf::BFLMeasConditionalPdf(const BFL::Gaussian& additiveNoise)
: BFL::AnalyticConditionalGaussianAdditiveNoise(additiveNoise, 2)
{
}

BFLMeasConditionalPdf::~BFLMeasConditionalPdf()
{
}

MatrixWrapper::ColumnVector BFLMeasConditionalPdf::ExpectedValueGet() const
{
    throw NotImplementedException();
    return MatrixWrapper::ColumnVector();
}

MatrixWrapper::Matrix BFLMeasConditionalPdf::dfGet(unsigned int i) const
{
    throw NotImplementedException();
    return MatrixWrapper::Matrix();
}
