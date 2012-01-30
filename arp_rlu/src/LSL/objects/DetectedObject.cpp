/*
 * DetectedObject.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "DetectedObject.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

DetectedObject::DetectedObject()
{
}

DetectedObject::~DetectedObject()
{
}

void DetectedObject::setScan(lsl::LaserScan)
{
    throw NotImplementedException();
    return;
}

Vector2 DetectedObject::getCartesianMean() const
{
    throw NotImplementedException();
    return Vector2();
}

Vector2 DetectedObject::getCartesianStddev() const
{
    throw NotImplementedException();
    return Vector2();
}

double DetectedObject::getApparentRange() const
{
    throw NotImplementedException();
    return -1.;
}

double DetectedObject::getApparentTheta() const
{
    throw NotImplementedException();
    return -1.;
}

void DetectedObject::computeStatistics()
{
    throw NotImplementedException();
    return;
}
