/*
 * EstimatedTwist2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedTwist2D.hpp>

using namespace arp_math;

EstimatedTwist2D::EstimatedTwist2D(Twist2D & _t, long double date, Eigen::Matrix<double,3,3> cov)
: Twist2D(_t)
, estimationDate(date)
, covariance(cov)
{
    ;
}

EstimatedTwist2D::EstimatedTwist2D(double _vx, double _vy, double _vh, long double date, Eigen::Matrix<double,3,3> cov)
: Twist2D(_vx, _vy, _vh)
, estimationDate(date)
, covariance(cov)
{
    ;
}

Eigen::Matrix<double,3,3> EstimatedTwist2D::cov() const
{
    return covariance;
}

long double EstimatedTwist2D::date() const
{
    return estimationDate;
}

long double& EstimatedTwist2D::dateRef()
{
    return estimationDate;
}

void EstimatedTwist2D::cov(Eigen::Matrix<double,3,3> _cov)
{
    covariance = _cov;
}

void EstimatedTwist2D::date(long double _date)
{
    estimationDate = _date;
}

EstimatedTwist2D EstimatedTwist2D::transport(Pose2D p) const
{
    Vector3 res =  p.inverse().getBigAdjoint()*getTVector();
    return EstimatedTwist2D();
}
