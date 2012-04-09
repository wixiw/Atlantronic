/*
 * EstimatedPose2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedPose2D.hpp>

using namespace arp_math;

EstimatedPose2D::EstimatedPose2D(Pose2D & _p)
: Pose2D(_p)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

EstimatedPose2D::EstimatedPose2D(Vector2 _translation, Rotation2 _orientation)
: Pose2D(_translation, _orientation)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

EstimatedPose2D::EstimatedPose2D(double _x, double _y, double _h)
: Pose2D(_x, _y, _h)
, estimationDate(0.)
, covariance(Eigen::Matrix<double,3,3>::Identity())
{
    ;
}

Eigen::Matrix<double,3,3> EstimatedPose2D::cov() const
{
    return covariance;
}

Eigen::Matrix<double,3,3>& EstimatedPose2D::covRef()
{
    return covariance;
}

double EstimatedPose2D::date() const
{
    return estimationDate;
}

double& EstimatedPose2D::dateRef()
{
    return estimationDate;
}

void EstimatedPose2D::cov(Eigen::Matrix<double,3,3> _cov)
{
    covariance = _cov;
}

void EstimatedPose2D::date(double _date)
{
    estimationDate = _date;
}
