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
    EstimatedTwist2D(Twist2D t);
    EstimatedTwist2D(double _vx = 0, double _vy = 0, double _vh = 0., long double date = 0., Eigen::Matrix<double,3,3> cov = Eigen::Matrix<double,3,3>::Identity());

    Eigen::Matrix<double,3,3> cov() const;
    long double date() const;
    long double& dateRef();

    void cov(Eigen::Matrix<double,3,3>) ;
    void date(long double);

    /**
     * Transporte et réduit le EstimatedTwist2D courant dans le nouveau repère
     * définit par la Pose2D du nouveau repère dans l'ancien.
     */
    EstimatedTwist2D transport(Pose2D p) const;

    private:
    Eigen::Matrix<double,3,3> covariance;
    long double estimationDate;
};
}

#endif /* _ARP_MATH_ESTIMATEDTWIST2D_HPP_ */
