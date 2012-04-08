/*
 * EstimatedTwist2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedTwist2D.hpp>

using namespace arp_math;

EstimatedTwist2D::EstimatedTwist2D(Twist2D & _t)
: Twist2D(_t)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

EstimatedTwist2D::EstimatedTwist2D(Vector2 _transVel, double _vh)
: Twist2D(_transVel, _vh)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

EstimatedTwist2D::EstimatedTwist2D(double _vx, double _vy, double _vh)
: Twist2D(_vx, _vy, _vh)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

Eigen::Matrix<double,3,3> EstimatedTwist2D::cov() const
{
    return covariance;
}

double EstimatedTwist2D::date() const
{
    return estimationDate;
}

double& EstimatedTwist2D::dateRef()
{
    return estimationDate;
}

void EstimatedTwist2D::cov(Eigen::Matrix<double,3,3> _cov)
{
    covariance = _cov;
}

void EstimatedTwist2D::date(double _date)
{
    estimationDate = _date;
}

Twist2D EstimatedTwist2D::toTwist()
{
    Twist2D t(vx(),vy(),vh());
    return t;
}
