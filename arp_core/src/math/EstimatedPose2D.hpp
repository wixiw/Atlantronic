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
class EstimatedPose2D: public Pose2D
{
    public:
        EstimatedPose2D(const Pose2D & p = Pose2D());
        EstimatedPose2D(const EstimatedPose2D & p);

        Covariance3 cov() const;
        Covariance3& covRef();
        long double date() const;
        long double& dateRef();

        void cov(const Covariance3 &);
        void date(const long double &);

        /**
         * Il s'agit du même opérateur que celui de Pose2D mais cette version déplace aussi la covariance de l'EstimatedPose2D.
         * @warning La covariance est toujours croissante ! Donc gare aux multiples changements de repère si on ne veut pas la dégrader
         * Ex :
         * Pose2D levier;
         * EstimatedPose2D estimpose1;
         * EstimatedPose2D estimpose2 = estimpose1 * levier;
         * // la covariance de estimpose2 a augmenté du fait levier (l'erreur d'orientation bave sur la translation)
         * EstimatedPose2D estimpose3 = estimpose2 * levier.inverse();
         * // l'espérance de estimpose3 sera identique à celle de estimpose1. En revanche, sa covariance sera encore
         * supérieure à estimpose2 (car on n'est pas plus précis quand on prend la canne à pêche par l'autre bout)
         */
        EstimatedPose2D operator*(const Pose2D& other) const;

        /**
         * Crée une chaine de caractère représentant la pose sous forme (x +- sigma, y +- sigma, theta +- sigma)
         */
        std::string toString() const;

    private:
        long double estimationDate;
        Covariance3 covariance;

};
}

#endif /* _ARP_MATH_ESTIMATEDPOSE2D_HPP_ */
