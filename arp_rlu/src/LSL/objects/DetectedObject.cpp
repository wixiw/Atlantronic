/*
 * DetectedObject.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "DetectedObject.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

DetectedObject::DetectedObject()
: associatedScan(LaserScan())
, apparentRange(0.)
, apparentTheta(0.)
, apparentTime(0.)
, cartMean(Vector2::Zero())
, cartStddev(Vector2::Zero())
{
}


DetectedObject::DetectedObject(const DetectedObject & d)
: associatedScan(d.getScan())
, apparentRange(d.getApparentRange())
, apparentTheta(d.getApparentTheta())
, apparentTime(d.getApparentTime())
, cartMean(d.getCartesianMean())
, cartStddev(d.getCartesianStddev())
{
}


DetectedObject::DetectedObject(const LaserScan & ls)
: associatedScan(ls)
, apparentRange(0.)
, apparentTheta(0.)
, apparentTime(0.)
, cartMean(Vector2::Zero())
, cartStddev(Vector2::Zero())
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

double DetectedObject::getApparentTime() const
{
    return apparentTime;
}

Vector2 DetectedObject::getApparentPointOfView() const
{
    return apparentPoV;
}

double DetectedObject::getApparentAngleOfView() const
{
    return apparentAoV;
}

void DetectedObject::computeStatistics()
{
    unsigned int n = associatedScan.getSize();
    if( n == 0 )
    {
        return;
    }

    if(!associatedScan.areCartesianDataAvailable())
    {
        associatedScan.computeCartesianData();
    }

    Eigen::MatrixXd cartdata = associatedScan.getCartesianData();

    double xMean = arp_math::mean(cartdata.row(1));
    double yMean = arp_math::mean(cartdata.row(2));
    double xStddev = arp_math::stddev(cartdata.row(1));
    double yStddev = arp_math::stddev(cartdata.row(2));

    cartMean = Vector2(xMean, yMean);
    cartStddev = Vector2(xStddev, yStddev);

    apparentTime = cartdata(0,(int)(n-1)/2);
    double xMed = cartdata(3,(int)(n-1)/2);
    double yMed = cartdata(4,(int)(n-1)/2);
    double hMed = cartdata(5,(int)(n-1)/2);
    apparentPoV = Vector2(xMed, yMed);
    apparentAoV = hMed;

    apparentRange = sqrt( (xMean-xMed)*(xMean-xMed) + (yMean-yMed)*(yMean-yMed) );
    apparentTheta = betweenMinusPiAndPlusPi( atan2( yMean-yMed, xMean-xMed ) - hMed );

    return;
}
