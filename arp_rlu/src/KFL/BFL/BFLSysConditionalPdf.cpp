/*
 * BFLSysConditionalPdf.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BFLSysConditionalPdf.hpp"

using namespace arp_rlu;
using namespace kfl;

BFLSysConditionalPdf::BFLSysConditionalPdf( const std::vector< MatrixWrapper::Matrix > &ratio, const BFL::Gaussian &additiveNoise)
: BFL::LinearAnalyticConditionalGaussian(ratio, additiveNoise)
{
}

BFLSysConditionalPdf::~BFLSysConditionalPdf()
{
}
