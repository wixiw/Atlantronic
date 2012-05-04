/*
 * Twist2D.hpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_TWIST2D_HPP_
#define _ARP_MATH_TWIST2D_HPP_

#include <math/math.hpp>
#include <math/Pose2D.hpp>
#include <iostream>

namespace arp_math
{
/*
 * \nonstableyet
 *
 * \class Twist2D
 *
 * \brief Torseur cinématique du robot
 *
 * Cette classe représente le troseur cinématique du robot par rapport à une référence externe.
 * Il s'agit de 3 trois double. Les 2 premiers représentent la vitesse de translation en mètres par seconde.
 * Le dernier correspond à la vitesse de rotation en radians par seconde.
 */

class Twist2D
{
    public:

        /** Constructeur principal.
         * Il permet d'initialiser les vitesses avec 3 double. */
        Twist2D(double _vx = 0,
                double _vy = 0,
                double _vh = 0);

        /** \returns la vitesse de translation selon l'axe x en m/s */
        double vx() const;

        /** \returns la vitesse de translation selon l'axe y en m/s */
        double vy() const;

        /** \returns la vitesse de rotation. Il s'agit d'un double en rad/sec */
        double vh() const;

        /** Pour le typekit */
        double& vxRef();
        double& vyRef();
        double& vhRef();

        /** \returns l'angle entre la vitesse de translation et l'axe x du repère = atan2(vy,vx)*/
        double speedAngle() const;

        /** \returns la norme du vecteur vitesse*/
        double speedNorm() const;

        /** \param _vx la vitesse de translation selon l'axe x en m/s */
        void vx(double _vx);

        /** \param _vy la vitesse de translation selon l'axe x en m/s */
        void vy(double _vy);

        /** \param _vh la vitesse de rotation en rad/sec */
        void vh(double _vh);

        /** affiche (vx,vy,vtheta) */
        std::string toString() const;

        /** Opérateur d'addition de deux Twist2*/
        Twist2D operator+(const Twist2D& b) const;

        /** Opérateur de soustraction de deux Twist2*/
        Twist2D operator-(const Twist2D& rhs) const;

        /** Opérateur de multiplication sclaire*/
        Twist2D operator*(const double & scalaire) const;

        /** Opérateur de division sclaire*/
        Twist2D operator/(const  double & scalaire) const;

        /** Soustraction sur place */
        Twist2D& operator-=(const Twist2D& _other);

        /** Addition sur place */
        Twist2D& operator+=(const Twist2D& _other);

        /** Opérateur d'égalité.
         * \param _other la Twist2D à comparer.
         * \returns vrai si les Twist2D sont exactement identiques
         * \remarks la comparaison est basée sur la double comparaison des
         * composantes de la partie translation et de l'angle de la partie
         * rotation. La précision utilisée est ici celle des double. */
        bool operator ==(Twist2D _other) const;

        bool operator !=(Twist2D other) const;

        /**
         * Retourne le Twist sous forme de Vecteur 3 (omega,vx,vy)
         */
        Vector3 getTVector() const;

        /**
         * Transporte et réduit le Twist courant dans le nouveau repère
         * définit par la Pose2D du nouveau repère dans l'ancien.
         */
        Twist2D transport(Pose2D p) const;

        /**
         * Calcul la distance entre 2 Twist.
         * @param twist : Twist2D dont on veut la distance par rapport à nous
         * @param coef : coefficients de ponderation pour le calcul de la distance
         * @return la distance
         */
        double distanceTo(Twist2D twist, double coefTrans, double coefRot) const;

    protected:
        Vector2 vitesseTranslation;
        double vitesseRotation;

};

std::ostream &operator<<( std::ostream &flux, arp_math::Twist2D const& t);


}

#endif /* _ARP_MATH_TWIST2D_HPP_ */
