/*
 * ObjectFinder.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */


#include "ObjectFinder.hpp"
#include <iostream>
#include <ros/console.h>

using namespace arp_rlu;
using namespace Eigen;


ObjectFinder::ObjectFinder() :
    mf(3)
{

}

ObjectFinder::~ObjectFinder()
{
}

void ObjectFinder::setPolarScan(Scan s)
{
    if (s.cols() == 0)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan is empty");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    if (s.rows() != 2)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan.rows() != 2 => Scan is not in polar representation ?");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    scan = MatrixXd(2, s.cols());
    scan = s;
}

Scan ObjectFinder::computeCartesianScan(double xOnTable, double yOnTable, double thetaOnTable)
{
    unsigned int n = scan.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder computeCartesianScan : Polar Scan is empty");
        return MatrixXd::Zero(0, 0);
    }

    Scan cartScan = MatrixXd(4, n);
    for (unsigned int i = 0; i < n; i++)
    {
        cartScan(0,i) = scan(0,i);
        cartScan(1,i) = scan(1,i);
        cartScan(2,i) = xOnTable + scan(1,i) * cos( scan(0,i) + thetaOnTable);
        cartScan(3,i) = yOnTable + scan(1,i) * sin( scan(0,i) + thetaOnTable);
    }
    scan = MatrixXd(4, n);
    scan = cartScan;
    return scan;
}
