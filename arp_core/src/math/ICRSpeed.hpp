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
 *  ro = atan (distance of the ICR). can be negative following side of the CIR with respect to the speed
 *  alpha = angle of speed at referential point. 0 if no speed at referential point
 *  q  = norm of speed at referential point + omega
 */

class ICRSpeed
{
    public:

        /** Constructeur principal.
         * Il permet une initialisation par défaut (à zéro) de la vitesse, avec un CIR à l'infini pour aller tout droit */
        ICRSpeed(double ro = PI/2.0, double alpha = 0.0,double q=0);

        /** Constructeur secondaire.
         * Il permet d'initialiser les vitesses avec un twist. */
        ICRSpeed(Twist2D twist);

        /** creation of ICRSpeed with a known ICR
         * if you have the ICR (don't work with translations)
         * ICR: point of the ICR
         * speedPoint: point where you give the speed
         * speed: speed vector at this point. the computation will consider that the speed is consistent (perpendicular to radius)
         */
        static ICRSpeed createFromICR(Vector2 ICR,Vector2 speedPoint, Vector2 speed);
        /** creation of ICRSpeed in case of a translation
         * alpha: angle of direction of translation
         * speed: speed of translation
         */
        static ICRSpeed createFromTranslation(double alpha,double speed);

        /** \returns atan(distance IRC) */
        double ro() const;

        /** \returns  angle of IRC to te referential*/
        double alpha() const;

        /** \returns v at referential+omega */
        double q() const;

        /** Pour le typekit */
        double& roRef();
        double& alphaRef();
        double& qRef();


        void ro(double ro);
        void alpha(double _alpha);
        void q(double q);

        /** returns a equivalent twist */
        Twist2D twist();

        /** affiche (vx,vy,vtheta) */
        std::string toString() const;


    protected:
        double m_ro;
        double m_alpha;
        double m_q;

};

std::ostream &operator<<( std::ostream &flux, arp_math::ICRSpeed const& t);

}

#endif /* _ARP_MATH_ICRSPEED_HPP_ */
