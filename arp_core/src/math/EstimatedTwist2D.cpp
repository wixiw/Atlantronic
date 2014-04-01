/*
 * EstimatedTwist2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedTwist2D.hpp>
#include <math/MathFactory.hpp>
#include <iostream>

using namespace arp_math;
using namespace arp_time;

EstimatedTwist2D::EstimatedTwist2D(const Twist2D & _t)
: Twist2D(_t)
, covariance(Covariance3::Identity())
, estimationDate(0.)
{
    ;
}

Covariance3 EstimatedTwist2D::cov() const
{
    return covariance;
}

ArdAbsoluteTime EstimatedTwist2D::date() const
{
    return estimationDate;
}

ArdAbsoluteTime& EstimatedTwist2D::dateRef()
{
    return estimationDate;
}

void EstimatedTwist2D::cov(const Covariance3 & _cov)
{
    covariance = _cov;
}

void EstimatedTwist2D::date(const ArdAbsoluteTime & _date)
{
    estimationDate = _date;
}

EstimatedTwist2D EstimatedTwist2D::transport(const Pose2D & p) const
{
    Vector3 v = p.inverse().getBigAdjoint()*getTVector();
    EstimatedTwist2D out( MathFactory::createTwist2DFromCartesianRepr( v ) );
    out.date( date() );
    out.cov( p.inverse().getBigAdjoint() * cov() * (p.inverse().getBigAdjoint()).transpose() );
    return out;
}

EstimatedTwist2D EstimatedTwist2D::changeProjection(const Pose2D & p) const
{
    Eigen::Matrix<double, 3, 3> M = Eigen::Matrix<double, 3, 3>::Identity();
    M.topLeftCorner<2,2>() = p.inverse().getRotationMatrix();
    M(2,2) = 1.;

    return MathFactory::createEstimatedTwist2DFromCartesianRepr( M * this->getTVector(),
                                                                 this->date(),
                                                                 M * this->cov() * M.transpose() );
}
