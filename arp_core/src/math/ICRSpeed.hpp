/*
 * ICRSpeed.hpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#ifndef _ARP_MATH_ICRSPEED_HPP_
#define _ARP_MATH_ICRSPEED_HPP_

#include <math/math.hpp>
#include <math/Pose2D.hpp>
#include <math/Twist2D.hpp>
#include <iostream>

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
 * Il s'agit de 3 trois double. Le premier est lié a la distance du CIR
 * Le deuxieme est lié à la direction du CIR par rapport au repere
 * Le 3eme exprime la vitesse du robot
 */

class ICRSpeed
{
    public:

        /** Constructeur principal.
         * Il permet une initialisation par défaut (à zéro) de la vitesse, avec un CIR à l'infini pour aller tout droit */
        ICRSpeed(double _ro = PI/2.0, double _alpha = PI/2.0,double _q=0);

        /** Constructeur secondaire.
         * Il permet d'initialiser les vitesses avec un twist. */
        ICRSpeed(Twist2D _twist);


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


        void ro(double _ro);
        void alpha(double _alpha);
        void q(double _q);

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
