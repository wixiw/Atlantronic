/*
 * BFLWrapper.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "BFLWrapper.hpp"

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
}

BFLWrapper::~BFLWrapper()
{
}

void BFLWrapper::init()
{
}
