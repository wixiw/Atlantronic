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
//    IM = np.zeros((2,1))
//    IM[0,0] = np.sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
//    IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])

    MatrixWrapper::ColumnVector X = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector beacon = ConditionalArgumentGet(1);

    MatrixWrapper::ColumnVector IM(2);
    IM(1) = sqrt( (X(1)-beacon(1))*(X(1)-beacon(1)) + (X(2)-beacon(1))*(X(2)-beacon(1)) );
    IM(2) = betweenMinusPiAndPlusPi( atan2(beacon(2) - X(2), beacon(1) - X(1) ) - X(3) );

    return IM;
}


MatrixWrapper::Matrix BFLMeasConditionalPdf::dfGet(unsigned int i) const
{
//    def J(X):
//        H = np.zeros( (2,3) )
//        H[0,0] = (X[0,0] - xBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
//        H[0,1] = (X[1,0] - yBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
//        H[0,2] = 0.
//        H[1,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
//        H[1,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
//        H[1,2] = -1.
//        return H

    MatrixWrapper::ColumnVector X = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector beacon = ConditionalArgumentGet(1);

    MatrixWrapper::Matrix H(2,3);
    H(1,1) = (X(1) - beacon(1)) / sqrt( (X(1) - beacon(1))*(X(1) - beacon(1)) + (X(2) - beacon(2))*(X(2) - beacon(2)) );
    H(1,2) = (X(2) - beacon(2)) / sqrt( (X(1) - beacon(1))*(X(1) - beacon(1)) + (X(2) - beacon(2))*(X(2) - beacon(2)) );
    H(1,3) = 0.;
    H(2,1) = (X(2) - beacon(2)) / ( (X(1) - beacon(1))*(X(1) - beacon(1)) + (X(2) - beacon(2))*(X(2) - beacon(2)) );
    H(2,2) = (X(1) - beacon(1)) / ( (X(1) - beacon(1))*(X(1) - beacon(1)) + (X(2) - beacon(2))*(X(2) - beacon(2)) );
    H(2,3) = -1.;

    return H;
}


