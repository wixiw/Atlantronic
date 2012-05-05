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
    EstimatedPose2D(const Pose2D & p = Pose2D());

    Covariance3 cov() const;
    Covariance3& covRef();
    long double date() const;
    long double& dateRef();

    void cov(const Covariance3 &) ;
    void date(const long double &);

    /**
     * Il s'agit du même opérateur que celui de Pose2D mais cette version déplace aussi la covariance de l'EstimatedPose2D
     */
    EstimatedPose2D operator*(const Pose2D& other) const;

    private:
    long double estimationDate;
    Covariance3 covariance;
};
}

#endif /* _ARP_MATH_ESTIMATEDPOSE2D_HPP_ */
