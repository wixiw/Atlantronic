/*
 * LaserScan.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "LaserScan.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

LaserScan::LaserScan()
: data(Eigen::MatrixXd::Zero(3,0))
{
}

LaserScan::LaserScan(const LaserScan & ls)
: data(ls.getPolarData())
{
}

unsigned int LaserScan::getSize() const
{
    throw NotImplementedException();
    return 0;
}

bool LaserScan::computeCartesianData(Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh)
{
    throw NotImplementedException();
    return false;
}

void LaserScan::setPolarData(Eigen::MatrixXd data)
{
    throw NotImplementedException();
    return;
}

Eigen::MatrixXd LaserScan::getPolarData() const
{
    throw NotImplementedException();
    return Eigen::MatrixXd(3,0);;
}

Eigen::MatrixXd LaserScan::getCartesianData() const
{
    throw NotImplementedException();
    return Eigen::MatrixXd(3,0);
}

Eigen::VectorXd LaserScan::getTimeData() const
{
    return Eigen::VectorXd(0);
}

bool LaserScan::areCartesianDataAvailable()
{
    throw NotImplementedException();
    return false;
}

unsigned int  LaserScan::cleanUp(double epsilon)
{
    throw NotImplementedException();
    return 0;
}
