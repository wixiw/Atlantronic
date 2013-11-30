/*
 * EstimatedICRSpeed.cpp
 *
 *  Created on: Nov 29, 2013
 *      Author: ard
 */

#include <math/EstimatedICRSpeed.hpp>
#include <math/MathFactory.hpp>
#include <iostream>

namespace arp_math
{

EstimatedICRSpeed::~EstimatedICRSpeed()
{
    // TODO Auto-generated destructor stub
}

EstimatedICRSpeed::EstimatedICRSpeed(const ICRSpeed & _t) :
        ICRSpeed(_t), covariance(Covariance3::Identity()), estimationDate(0.)
{

}

Covariance3 EstimatedICRSpeed::cov() const
{
    return covariance;
}

long double EstimatedICRSpeed::date() const
{
    return estimationDate;
}

long double& EstimatedICRSpeed::dateRef()
{
    return estimationDate;
}

void EstimatedICRSpeed::cov(const Covariance3 & _cov)
{
    covariance = _cov;
}

void EstimatedICRSpeed::date(const long double & _date)
{
    estimationDate = _date;
}

EstimatedTwist2D EstimatedICRSpeed::twist() const
{
    Twist2D twist = ICRSpeed::twist();
    EstimatedTwist2D estimatedTwist(twist);
    //TODO convertir les covariances
    estimatedTwist.date(date());
    return estimatedTwist;
}


EstimatedICRSpeed EstimatedICRSpeed::transport(const Pose2D & p) const
{
    //TODO a implementer
    return EstimatedICRSpeed();
}

EstimatedICRSpeed EstimatedICRSpeed::changeProjection(const Pose2D & p) const
{
    //TODO a implementer
    return EstimatedICRSpeed();
}

} /* namespace arp_math */
