/*
 * EstimatedTwist2D.hpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_ESTIMATEDTWIST2D_HPP_
#define _ARP_MATH_ESTIMATEDTWIST2D_HPP_

#include <math/math.hpp>
#include <math/Twist2D.hpp>

namespace arp_math
{
class EstimatedTwist2D : public Twist2D
{
    public:
    EstimatedTwist2D(Twist2D &);
    EstimatedTwist2D(Vector2 _transVel = Vector2(0., 0.), double _vh = 0.);
    EstimatedTwist2D(double _vx, double _vy, double _vh = 0.);

    Eigen::Matrix<double,3,3> cov() const;
    double date() const;
    double& dateRef();

    void cov(Eigen::Matrix<double,3,3>) ;
    void date(double);

    private:
    Eigen::Matrix<double,3,3> covariance;
    double estimationDate;
};
}

#endif /* _ARP_MATH_ESTIMATEDTWIST2D_HPP_ */
