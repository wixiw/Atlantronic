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
{
}

LaserScan::~LaserScan()
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

bool LaserScan::areCartesianDataAvailaible()
{
    throw NotImplementedException();
    return false;
}

bool LaserScan::cleanUp()
{
    throw NotImplementedException();
    return false;
}
