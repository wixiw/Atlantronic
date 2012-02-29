/*
 * DetectedCircle.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "DetectedCircle.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

DetectedCircle::DetectedCircle()
: Circle()
, DetectedObject()
, apparentCenterRange(0.)
, apparentCenterTheta(0.)
, apparentCenterTime(0.)
{
}

DetectedCircle::DetectedCircle(const DetectedObject & ls)
: Circle()
, DetectedObject(ls)
, apparentCenterRange(0.)
, apparentCenterTheta(0.)
, apparentCenterTime(0.)
{
}

double DetectedCircle::getApparentCenterRange() const
{
    return apparentCenterRange;
}

double DetectedCircle::getApparentCenterTheta() const
{
    return apparentCenterTheta;
}

double DetectedCircle::getApparentCenterTime() const
{
    return apparentCenterTime;
}

void DetectedCircle::setApparentCenterRange( double r)
{
    apparentCenterRange = r;
}

void DetectedCircle::setApparentCenterTheta( double t)
{
    apparentCenterTheta = t;
}

void DetectedCircle::setApparentCenterTime( double t)
{
    apparentCenterTime = t;
}
