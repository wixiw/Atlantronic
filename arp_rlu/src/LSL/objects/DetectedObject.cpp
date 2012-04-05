/*
 * DetectedObject.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "DetectedObject.hpp"

#include "LSL/Logger.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

DetectedObject::DetectedObject()
: associatedScan(LaserScan())
, apparentCartMeanRange(0.)
, apparentCartMeanTheta(0.)
, apparentCartMeanTime(0.)
, PoV(Vector2::Zero())
, AoV(0.)
, cartMean(Vector2::Zero())
, cartStddev(Vector2::Zero())
{
}


DetectedObject::DetectedObject(const DetectedObject & d)
: associatedScan(d.getScan())
, apparentCartMeanRange(d.getApparentCartesianMeanRange())
, apparentCartMeanTheta(d.getApparentCartesianMeanTheta())
, apparentCartMeanTime(d.getApparentCartesianMeanTime())
, PoV(d.getPointOfView())
, AoV(d.getAngleOfView())
, cartMean(d.getCartesianMean())
, cartStddev(d.getCartesianStddev())
{
    this->computeStatistics();
}


DetectedObject::DetectedObject(const LaserScan & ls)
: associatedScan(ls)
, apparentCartMeanRange(0.)
, apparentCartMeanTheta(0.)
, apparentCartMeanTime(0.)
, PoV(Vector2::Zero())
, AoV(0.)
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

double DetectedObject::getApparentCartesianMeanRange() const
{
    return apparentCartMeanRange;
}

double DetectedObject::getApparentCartesianMeanTheta() const
{
    return apparentCartMeanTheta;
}

double DetectedObject::getApparentCartesianMeanTime() const
{
    return apparentCartMeanTime;
}

Vector2 DetectedObject::getPointOfView() const
{
    return PoV;
}

double DetectedObject::getAngleOfView() const
{
    return AoV;
}

void DetectedObject::computeStatistics()
{
    unsigned int n = associatedScan.getSize();
    if( n == 0 )
    {
        Log( WARN ) << "DetectedObject::computeStatistics" << " - " << "LaserScan is empty => Return";
        return;
    }

    if(!associatedScan.areCartesianDataAvailable())
    {
        Log( NOTICE ) << "DetectedObject::computeStatistics" << " - " << "cartesian data are not available => compute it with trivial position";
        associatedScan.computeCartesianData();
    }

    Eigen::MatrixXd cartdata = associatedScan.getCartesianData();

    double xMean = arp_math::mean(cartdata.row(1));
    double yMean = arp_math::mean(cartdata.row(2));
    double xStddev = arp_math::stddev(cartdata.row(1));
    double yStddev = arp_math::stddev(cartdata.row(2));

    cartMean = Vector2(xMean, yMean);
    cartStddev = Vector2(xStddev, yStddev);

    apparentCartMeanTime = cartdata(0,(int)(n-1)/2);
    double xMed = cartdata(3,(int)(n-1)/2);
    double yMed = cartdata(4,(int)(n-1)/2);
    double hMed = cartdata(5,(int)(n-1)/2);
    PoV = Vector2(xMed, yMed);
    AoV = hMed;

    apparentCartMeanRange = sqrt( (xMean-xMed)*(xMean-xMed) + (yMean-yMed)*(yMean-yMed) );
    apparentCartMeanTheta = betweenMinusPiAndPlusPi( atan2( yMean-yMed, xMean-xMed ) - hMed );

    return;
}
