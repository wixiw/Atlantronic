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
        ICRSpeed(double ro, ICR ICR);

        /** Constructeur secondaire.
         * Il permet d'initialiser les vitesses avec un twist. */
        ICRSpeed(Twist2DNorm twist);
        ICRSpeed(Twist2D twist);

        /** returns the direction (in 3D) of the speed) */
        Vector3 speedDirection();
        /** returns equivalent representation with opposite sign of ro and ICR at antipod*/
        ICRSpeed getOppositeRep();

        /** for a robot not moving, allow to create the ICRSpeed defined by the position "ICR" and a null speed*/
        static ICRSpeed createIdleFromICRPosition(Vector2 ICRPosition);
        /** for a robot not moving, allow to create the ICRSpeed defined by the ICR at infinity with angle "angle" and a null speed*/
        static ICRSpeed createIdleFromTranslation(double angle);

        double ro() const;
        double phi() const;
        double delta() const;

        /** for typekit */
        double& roRef();
        double& phiRef();
        double& deltaRef();

        void ro(double ro);
        void phi(double phi);
        void delta(double q);

        /** returns a equivalent twist */
        Twist2DNorm twistNorm();
        Twist2D twist();

        /** affiche (vx,vy,vtheta) */
        std::string toString() const;

    protected:
        double m_ro;
        ICR m_ICR;
        void initFromTwist(Twist2DNorm twist);


};

std::ostream &operator<<(std::ostream &flux, arp_math::ICRSpeed const& t);

}

#endif /* _ARP_MATH_ICRSPEED_HPP_ */
