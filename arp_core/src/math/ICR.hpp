/*
 * ICRSpeed.hpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#ifndef _ARP_MATH_ICR_HPP_
#define _ARP_MATH_ICR_HPP_

#include "math/math.hpp"
#include "math/Pose2D.hpp"
#include "math/Twist2DNorm.hpp"
#include <string.h>

namespace arp_math
{
/*
 * \nonstableyet
 *
 * \class ICR
 *
 * \brief position du CIR du robot, exprimé en sphérique, au référentiel du robot
 *
 */

class ICR
{
    static const double epsilon = 1E-6;

    public:

        /** Constructeur principal.
         * Il permet une initialisation par défaut avec un CIR à l'infini (pour aller tout droit) */
        ICR(double phi = 0.0, double delta = 0);
        /** construction from a speed vector expressed in cartesian coordinates */
        ICR(Vector3 cartesianVector);
        /** copy constructor */
        ICR(const ICR&);

        /** returns ICR on the other side of the sphere */
        ICR getAntipodICR() const;
        /** returns the orthodromic distance to another ICR on the sphere */
        double sphericalDistance(const ICR& ICR2) const;
        /** returns a cartesian vector instead of spherical angles (its norm is 1)*/
        Vector3 getCartesianVector() const;
        /** returns an ICR, on the great circle between me and ICR2, that is at a distance s from me*/
        ICR getIntermediate(const ICR& ICR2, double s) const;
        /** Transport the ICR coordinate to a new reference. p is the position of the new reference is the current coordinates*/
        ICR transport(const Pose2D & p) const;
        /** Same as previous but publish internal computations */ //TODO this params must have a physical sense... but which one ?
        ICR transport(const Pose2D & p, double& a, double &b, double& c) const;

        double phi() const;
        double delta() const;

        /** Pour le typekit */
        double& phiRef();
        double& deltaRef();

        void phi(double phi);
        void delta(double q);

        /** affiche le resume*/
        std::string toString() const;

    protected:
        double m_phi;
        double m_delta;

};

std::ostream &operator<<(std::ostream &flux, arp_math::ICR const& t);

}

#endif /* _ARP_MATH_ICR_HPP_ */
