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
    EstimatedTwist2D(Twist2D t = Twist2D());

    Covariance3 cov() const;
    long double date() const;
    long double& dateRef();

    void cov(Covariance3) ;
    void date(long double);

    /**
     * Transporte et réduit le EstimatedTwist2D courant dans le nouveau repère
     * définit par la Pose2D du nouveau repère dans l'ancien.
     */
    EstimatedTwist2D transport(Pose2D p) const;

    private:
    Covariance3 covariance;
    long double estimationDate;
};
}

#endif /* _ARP_MATH_ESTIMATEDTWIST2D_HPP_ */
