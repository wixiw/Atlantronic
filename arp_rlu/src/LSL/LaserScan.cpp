/*
 * LaserScan.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "LaserScan.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;
using namespace arp_core::log;

LaserScan::LaserScan()
: data(Eigen::MatrixXd::Zero(3,0))
{
}

LaserScan::LaserScan(const LaserScan & ls)
: data(ls.getPolarData())
{
    if(ls.areCartesianDataAvailable())
    {
        Eigen::MatrixXd cart = ls.getCartesianData();
        Eigen::MatrixXd newdata = Eigen::MatrixXd::Zero(8,cart.cols());
        newdata.topRows(3) = data;
        newdata.bottomRows(5) = cart.bottomRows(5);
        data = newdata;
    }
}

unsigned int LaserScan::getSize() const
{
    return data.cols();
}

bool LaserScan::computeCartesianData(Eigen::VectorXd ttc, Eigen::VectorXd xxc, Eigen::VectorXd yyc, Eigen::VectorXd hhc)
{
    Eigen::VectorXd tt = getTimeData();
    int n = tt.size();
    if (n == 0)
    {
        Log( WARN ) << "LaserScan::computeCartesianData" << " - " << "LaserScan is empty => Return false";
        return false;
    }

    if( ttc.size() != xxc.size() ||
            ttc.size() != yyc.size() ||
            ttc.size() != hhc.size() )
    {
        Log( ERROR ) << "LaserScan::computeCartesianData" << " - " << "ttc, xxc, yyc and hhc do not have same lenght => Return false";
        return false;
    }

    Eigen::VectorXd xx;
    Eigen::VectorXd yy;
    Eigen::VectorXd hh;
    if( ttc.size() == 0 )
    {
        Log( INFO ) << "LaserScan::computeCartesianData" << " - " << "ttc is empty => using zero-vector for xx, yy and hh";
        xx.setZero(n);
        yy.setZero(n);
        hh.setZero(n);
    }
    else
    {
        xx = Interpolator::transInterp(tt, ttc, xxc);
        yy = Interpolator::transInterp(tt, ttc, yyc);
        hh = Interpolator::rotInterp(tt, ttc, hhc);
    }

    if( xx.size() != n || hh.size() != n || hh.size() != n )
    {
        Log( ERROR ) << "LaserScan::computeCartesianData" << " - " << "Error seems occured during interpolation => Return false";
        return false;
    }

    Eigen::MatrixXd newdata(8, n);
    newdata.topRows(3) = data.topRows(3);
    for(int i = 0; i < n; i++)
    {
        newdata(3, i) = xx[i] + data(1, i) * cos(data(2, i) + hh[i]);
        newdata(4, i) = yy[i] + data(1, i) * sin(data(2, i) + hh[i]);
        newdata(5, i) = xx[i];
        newdata(6, i) = yy[i];
        newdata(7, i) = hh[i];
    }
    data = newdata;
    return true;
}

void LaserScan::setPolarData(Eigen::MatrixXd data_)
{
    data = data_;
    return;
}

Eigen::MatrixXd LaserScan::getPolarData() const
{
    if(getSize() == 0)
    {
        Log( NOTICE ) << "LaserScan::getPolarData" << " - " << "LaserScan is empty => Return MatrixXd::Zero(3,0)";
        return Eigen::MatrixXd::Zero(3,0);
    }
    return data.topRows(3);
}

Eigen::MatrixXd LaserScan::getCartesianData() const
{
    if(!this->areCartesianDataAvailable())
    {
        Log( NOTICE ) << "LaserScan::getCartesianData" << " - " << "cartesian data are not available => Return MatrixXd::Zero(6,0)";
        return Eigen::MatrixXd::Zero(6,0);
    }
    Eigen::MatrixXd cart = Eigen::MatrixXd::Zero(6,data.cols());
    cart.topRows(1) = data.topRows(1);
    cart.bottomRows(5) = data.bottomRows(5);
    return cart;
}

Eigen::VectorXd LaserScan::getTimeData() const
{
    return data.row(0);
}

bool LaserScan::areCartesianDataAvailable() const
{
    return (data.rows() == 8);
}

unsigned int  LaserScan::cleanUp(double epsilon)
{
    unsigned int N = 0;
    for(int i = 0 ; i < data.cols() ; i++)
    {
        if(data(1,i) > epsilon)
            N++;
    }
    Eigen::MatrixXd newdata(data.rows(), N);
    unsigned int k = 0;
    for(int i = 0 ; i < data.cols() ; i++)
    {
        if(data(1,i) > epsilon)
        {
            newdata.col(k) = data.col(i);
            k++;
        }
    }
    N = data.cols() - N;
    data = newdata;
    return N;
}
