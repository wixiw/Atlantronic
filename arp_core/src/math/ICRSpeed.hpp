/*
 * ICRSpeed.hpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#ifndef _ARP_MATH_ICRSPEED_HPP_
#define _ARP_MATH_ICRSPEED_HPP_

#include "math/math.hpp"
#include "math/Pose2D.hpp"
#include "math/Twist2D.hpp"
#include "math/Twist2DNorm.hpp"
#include "math/ICR.hpp"
#include <string.h>

namespace arp_math
{
/*
 * \nonstableyet
 *
 * \class ICRSpeed
 *
 * \brief Torseur cinématique du robot, exprimé à l'aide du CIR
 *
 * Cette classe représente le torseur cinématique du robot par rapport à une référence externe.
 * Il s'agit de 3 trois double.
 *
 *  ro = atan (distance of the ICR). always positive
 *  alpha = angle of line to CIR. 0 if no speed at referential point
 *  q  = norm of speed at referential point + omega
 */

class ICRSpeed
{
    public:

        /** Constructeur principal.
         * Il permet une initialisation par défaut (à zéro) de la vitesse, avec un CIR à l'infini pour aller tout droit */
        ICRSpeed(double ro = 0.0, double phi = 0.0, double delta = 0.0);
        ICRSpeed(double ro, const ICR& ICR);
        ICRSpeed(const ICRSpeed& icrSpeed);

        /** Constructeur secondaire.
         * Il permet d'initialiser les vitesses avec un twist. */
        ICRSpeed(const Twist2DNorm& twist);
        ICRSpeed(const Twist2D& twist);


        /** returns the direction (in 3D) of the speed) */
        Vector3 speedDirection() const;
        /** returns equivalent representation with opposite sign of ro and ICR at antipod*/
        ICRSpeed getOppositeRep() const;
        /** normalize the representation so that ro>0 */
        ICRSpeed getNormalizedRep() const;

        /** for a robot not moving, allow to create the ICRSpeed defined by the position "ICR" and a null speed*/
        static ICRSpeed createIdleFromICRPosition(const Vector2& ICRPosition);
        /** for a robot not moving, allow to create the ICRSpeed defined by the ICR at infinity with angle "angle" and a null speed*/
        static ICRSpeed createIdleFromTranslation(double angle);

        /**
         * Transporte ET réduit l'ICRSpeed courant dans le nouveau repère
         * définit par la Pose2D du nouveau repère dans l'ancien.\n
         * Le repère de projection ET de réduction sont modifiés :\n
         */
        ICRSpeed transport(const Pose2D & p) const;


        double ro() const;
        double phi() const;
        double delta() const;
        ICR getICR() const;

        /** for typekit */
        double& roRef();
        double& phiRef();
        double& deltaRef();

        void ro(double ro);
        void phi(double phi);
        void delta(double q);

        /** returns a equivalent twist */
        Twist2DNorm twistNorm() const;
        Twist2D twist() const;

        double distanceTo(ICRSpeed other, double coefTrans, double coefRot) const;

        /** Compute the norm of the translation speed |v|| = sqrt(vx*vx+vy*vy) = |ro()*cos(delta)|*sqrt(cos2(phi()) + sin2(phi())) = |ro()*cos(delta())|  */
        double getTranslationSpeedNorm();

        /** affiche (vx,vy,vtheta) */
        std::string toString() const;

        /** Opérateur d'égalité.
         * \param _other la ICRSpeed à comparer.
         * \returns vrai si les ICRSpeed (ou leurs opposés) sont exactement identiques
         * \remarks la comparaison est basée sur la comparaison des
         * composantes ro,delta et phi entre elles.
         * La précision utilisée est ici celle des double. */
        bool operator ==(const ICRSpeed& other) const;

        bool operator !=(const ICRSpeed& other) const;

    protected:
        double m_ro;
        ICR m_ICR;
        void initFromTwist(const Twist2DNorm& twist);


};

std::ostream& operator<<(std::ostream& flux, arp_math::ICRSpeed const& t);

}

#endif /* _ARP_MATH_ICRSPEED_HPP_ */
