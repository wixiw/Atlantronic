/*
 * Twist2.hpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#ifndef TWIST2_HPP_
#define TWIST2_HPP_

#include "Geometry.hpp"

namespace arp_math
{

    /** \ingroup ARPMath
     * \nonstableyet
     *
     * \class Twist2
     *
     * \brief Torseur cinématique du robot
     *
     * Cette classe représente le troseur cinématique du robot par rapport à une référence externe.
     * Il s'agit de 3 trois double. Les 2 premiers représentent la vitesse de translation en mètres par seconde.
     * Le dernier correspond à la vitesse de rotation en radians par seconde.
     */
    class Twist2
    {
    public:
        /** Constructeur par copie */
        Twist2(Twist2 & _ref);

        /** Constructeur principal.
         * Il permet une initialisation par défaut (à zéro) de la vitesse */
        Twist2(Vector2 _vitesseTranslation = Vector2(0, 0),
                double _vitesseRotation = 0);

        /** Destructeur debaze */
        virtual ~Twist2();

        /** \returns la vitesse de translation. Il s'agit d'un Eigen::Vector de taille 2 en m/s */
        Vector2 VTrans();

        /** \returns la vitesse de rotation. Il s'agit d'un double en rad/sec */
        double VRot();

        /** \param _v la vitesse de translation. Il s'agit d'un Eigen::Vector de taille 2 en m/s */
        void VTrans(Vector2 _v);

        /** \param _omega la vitesse de rotation. Il s'agit d'un double en rad/sec */
        void VRot(double _omega);

    protected:
        Vector2 vitesseTranslation;
        double vitesseRotation;

    public:
        /** Opérateur d'addition de deux Twist2*/
        inline friend Twist2 operator+(const Twist2& lhs, const Twist2& rhs);

        /** Opérateur de soustraction de deux Twist2*/
        inline friend Twist2 operator-(const Twist2& lhs, const Twist2& rhs);

        /** Soustraction sur place */
        inline Twist2& operator-=(const Twist2& _other);

        /** Addition sur place */
        inline Twist2& operator+=(const Twist2& _other);

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        inline double& operator()(int i);

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        double operator[](int index) const;

        /** Accession aux composants via des index.
         * Index(0..1) donne la vitesse de translation
         * Index(3) donne la vitesse de rotation */
        double& operator[](int index);
    };

}

#endif /* TWIST2_HPP_ */
