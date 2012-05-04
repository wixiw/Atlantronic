/*
 * EstimatedPose2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedPose2D.hpp>

using namespace arp_math;

EstimatedPose2D::EstimatedPose2D(Pose2D _p)
: Pose2D(_p)
, estimationDate(0.)
, covariance(Covariance3::Identity())
{
    ;
}

Covariance3 EstimatedPose2D::cov() const
{
    return covariance;
}

Covariance3& EstimatedPose2D::covRef()
{
    return covariance;
}

long double EstimatedPose2D::date() const
{
    return estimationDate;
}

long double& EstimatedPose2D::dateRef()
{
    return estimationDate;
}

void EstimatedPose2D::cov(Covariance3 _cov)
{
    covariance = _cov;
}

void EstimatedPose2D::date(long double _date)
{
    estimationDate = _date;
}
