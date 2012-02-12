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
: associatedScan(LaserScan())
, apparentRange(0.)
, apparentTheta(0.)
, cartMean(0.)
, cartStddev(0.)
{
}


DetectedObject::DetectedObject(const DetectedObject & d)
: associatedScan(d.getScan())
, apparentRange(d.getApparentRange())
, apparentTheta(d.getApparentTheta())
, cartMean(d.getCartesianMean())
, cartStddev(d.getCartesianStddev())
{
}


DetectedObject::DetectedObject(const LaserScan & ls)
: associatedScan(ls)
, apparentRange(0.)
, apparentTheta(0.)
, cartMean(0.)
, cartStddev(0.)
{
    this->computeStatistics();
}

void DetectedObject::setScan(lsl::LaserScan ls)
{
    associatedScan = ls;
    computeStatistics();
    return;
}


LaserScan DetectedObject::getScan() const
{
    return associatedScan;
}

Vector2 DetectedObject::getCartesianMean() const
{
    return cartMean;
}

Vector2 DetectedObject::getCartesianStddev() const
{
    return cartStddev;
}

double DetectedObject::getApparentRange() const
{
    return apparentRange;
}

double DetectedObject::getApparentTheta() const
{
    return apparentTheta;
}

void DetectedObject::computeStatistics()
{
    throw NotImplementedException();
    return;
}
