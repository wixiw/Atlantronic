/*
 * EstimatedPose2D.hpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_ESTIMATEDPOSE2D_HPP_
#define _ARP_MATH_ESTIMATEDPOSE2D_HPP_

#include <math/math.hpp>
#include <math/Pose2D.hpp>

namespace arp_math
{
class EstimatedPose2D : public Pose2D
{
    public:
    EstimatedPose2D(Pose2D &);
    EstimatedPose2D(Vector2 _translation = Vector2(0., 0.), Rotation2 _orientation = Rotation2(0.));
    EstimatedPose2D(double _x, double _y, double _h = 0.);

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

#endif /* _ARP_MATH_ESTIMATEDPOSE2D_HPP_ */
