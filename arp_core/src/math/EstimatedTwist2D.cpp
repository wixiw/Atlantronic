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

EstimatedTwist2D::EstimatedTwist2D(Twist2D _t)
: Twist2D(_t)
{
    ;
}

Covariance3 EstimatedTwist2D::cov() const
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

void EstimatedTwist2D::cov(Covariance3 _cov)
{
    covariance = _cov;
}

void EstimatedTwist2D::date(long double _date)
{
    estimationDate = _date;
}

EstimatedTwist2D EstimatedTwist2D::transport(Pose2D p) const
{
    Vector3 v = p.inverse().getBigAdjoint()*getTVector();
    EstimatedTwist2D out( MathFactory::createTwist2DFromCartesianRepr( v ) );
    out.date( date() );

    std::cout << "cov:\n" << cov() << std::endl;
    std::cout << "p:\n (" << p.toString() << std::endl;
    std::cout << "p.inverse():\n" << p.inverse().toString() << std::endl;
    std::cout << "p.inverse().getBigAdjoint():\n" << p.inverse().getBigAdjoint() << std::endl;
    std::cout << "(p.inverse().getBigAdjoint()).inverse():\n" << (p.inverse().getBigAdjoint()).inverse() << std::endl;
    std::cout << "new cov:\n" << p.inverse().getBigAdjoint() * cov() * (p.inverse().getBigAdjoint()).inverse() << std::endl;

    out.cov( p.inverse().getBigAdjoint() * cov() * (p.inverse().getBigAdjoint()).inverse() );
    return out;
}
