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

namespace arp_math
{

/** \ingroup arp_math
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
         * Il permet une initialisation par défaut (à zéro) de la vitesse */
        Twist2D(Vector2 _vitesseTranslation = Vector2(0, 0),
                double _vitesseRotation = 0);

        /** Constructeur secondaire.
         * Il permet d'initialiser les vitesses avec 3 double. */
        Twist2D(double _vx,
                double _vy,
                double _vh = 0);

        /**
         * Permet de construire le Twist à partir de sa représentation vectorielle (omega, vx, vy)
         */
        Twist2D(Vector3 T);

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
        inline friend Twist2D operator+(const Twist2D& lhs, const Twist2D& rhs);

        /** Opérateur de soustraction de deux Twist2*/
        inline friend Twist2D operator-(const Twist2D& lhs, const Twist2D& rhs);

        /** Soustraction sur place */
        inline Twist2D& operator-=(const Twist2D& _other);

        /** Addition sur place */
        inline Twist2D& operator+=(const Twist2D& _other);

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        inline double& operator()(int i);

        /** Opérateur d'égalité.
         * \param _other la Twist2D à comparer.
         * \returns vrai si les Twist2D sont exactement identiques
         * \remarks la comparaison est basée sur la double comparaison des
         * composantes de la partie translation et de l'angle de la partie
         * rotation. La précision utilisée est ici celle des double. */
        bool operator ==(Twist2D _other) const;

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        double operator[](int index) const;

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        double& operator[](int index);

        /**
         * Retourne le Twist sous forme de Vecteur 3 (omega,vx,vy)
         */
        Vector3 getTVector() const;

        /**
         * Transporte et réduit le Twist courant dans le nouveau repère
         * définit par la Pose2D du nouveau repère dans l'ancien.
         */
        Twist2D transport(Pose2D p) const;

    protected:
        Vector2 vitesseTranslation;
        double vitesseRotation;

};

}

#endif /* _ARP_MATH_TWIST2D_HPP_ */
