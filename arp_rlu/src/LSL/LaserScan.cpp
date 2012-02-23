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
    if(ls.areCartesianDataAvailable())
    {
        Eigen::MatrixXd cart = ls.getCartesianData();
        Eigen::MatrixXd newdata = Eigen::MatrixXd::Zero(5,cart.cols());
        newdata.topRows(3) = data;
        newdata.bottomRows(2) = cart.bottomRows(2);
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
        return false;
    }

    if( ttc.size() != xxc.size() ||
            ttc.size() != yyc.size() ||
            ttc.size() != hhc.size() )
    {
        return false;
    }

    Eigen::VectorXd xx;
    Eigen::VectorXd yy;
    Eigen::VectorXd hh;
    if( ttc.size() == 0 )
    {
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
        return false;
    }

    Eigen::MatrixXd newdata(5, n);
    newdata.topRows(3) = data.topRows(3);
    for(int i = 0; i < n; i++)
    {
        newdata(3, i) = xx[i] + data(1, i) * cos(data(2, i) + hh[i]);
        newdata(4, i) = yy[i] + data(1, i) * sin(data(2, i) + hh[i]);
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
        return Eigen::MatrixXd::Zero(3,0);
    }
    return data.topRows(3);
}

Eigen::MatrixXd LaserScan::getCartesianData() const
{
    if(!this->areCartesianDataAvailable())
        return Eigen::MatrixXd::Zero(3,0);
    Eigen::MatrixXd cart = Eigen::MatrixXd::Zero(3,data.cols());
    cart.topRows(1) = data.topRows(1);
    cart.bottomRows(2) = data.bottomRows(2);
    return cart;
}

Eigen::VectorXd LaserScan::getTimeData() const
{
    return data.row(0);
}

bool LaserScan::areCartesianDataAvailable() const
{
    return (data.rows() == 5);
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
