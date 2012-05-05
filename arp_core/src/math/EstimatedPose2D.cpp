/*
 * EstimatedPose2D.cpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */


#include <math/EstimatedPose2D.hpp>
#include <math/MathFactory.hpp>

using namespace arp_math;

EstimatedPose2D::EstimatedPose2D(const Pose2D & _p)
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

void EstimatedPose2D::cov(const Covariance3 & _cov)
{
    covariance = _cov;
}

void EstimatedPose2D::date(const long double & _date)
{
    estimationDate = _date;
}



EstimatedPose2D EstimatedPose2D::operator*(const Pose2D& H) const
{
    Pose2D expect = MathFactory::createPose2D( this->getRotationMatrix() * H.translation() + this->translation() ,
                                               this->orientation() * H.orientation() );

    EstimatedPose2D out(expect);
    out.date( this->date() );

    // Calcul des Jacobiennes de f(theta)
    Vector2 Jx( -sin(this->angle()) ,  cos(this->angle()) );
    Vector2 Jy( -cos(this->angle()) , -sin(this->angle()) );

    Covariance3 cov = Covariance3::Identity();
    cov.topLeftCorner<2,2>() = Jx * Jx.transpose() * H.x() * H.x() * this->cov()(2,2)
                             + Jy * Jy.transpose() * H.y() * H.y() * this->cov()(2,2)
                             + this->cov().topLeftCorner<2,2>();
    cov(2,2) = this->cov()(2,2);
    out.cov( cov );
    return out;
}
