/*
 * Pose.hpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

// TODO BMO :
// * faire compiler les operateurs << - +
// * utiliser les références et les const
// * debuger matrix4 (??!?)
// * opérateurs approximation et différence
// * faire apparaitre cette todolist dans le Doxygen

#ifndef _ARPMATH_POSE_HPP_
#define _ARPMATH_POSE_HPP_

#include "Geometry.hpp"

namespace arp_math
{

    /** \ingroup arp_math
     * \nonstableyet
     *
     * \class Pose
     *
     * \brief Pose du robot
     *
     * Cette classe représente la pose (position 2D et cap) du robot par rapport à une référence externe.
     * Les unités sont les mètres et les radians.
     */
    class Pose
    {
    public:
        /** Constructeur par copie */
        Pose(Pose & _pose);

        /** Constructeur principal.
         * Il permet une initialisation par défaut de la Pose */
        Pose(Vector2 _translation = Vector2(0, 0),
                Rotation2 _orientation = Rotation2(0));

        /** \returns Destructeur debaze */
        ~Pose();

        // Getters
        /** \returns la partie translation. Il s'agit d'un Eigen::Vector de taille 2 en m */
        Vector2 translation();

        /** \returns la partie rotation de la Pose (en radians). */
        Rotation2 rotation2();

        //Displacement displacement();
        /** \returns la composante x de la partie translation en m */
        double x();

        /** \returns la composante y de la partie translation en m */
        double y();

        /** \returns un double image du cap en radians */
        double angle();

        /** \returns la matrice homogène 3x3. Le bloc haut gauche est constitué de la matrice orthogonale 2x2
         * de la rotation. Le vecteur colonne 2x1 du bloc haut droit est constitué du vecteur translation */
        Eigen::Matrix<double, 3, 3> matrix3();

        //	Eigen::Matrix<double,4,4> matrix4();

        // Setters
        /** Permet de modifier la partie translation.
         * \param _translation Vector2 qui correspond à la partie translation que l'on souhaite en m */
        void translation(Vector2 _translation);

        /** Permet de modifier la partie rotation.
         * \param _orientation Rotation2 qui correspond à la partie rotation que l'on souhaite */
        void rotation2(Rotation2 _orientation);

        /** Permet de modifier la première composante de la partie translation.
         * \param double correspondant à la première composante de la partie translation en m */
        void x(double _x);

        /** Permet de modifierARPMath la seconde composante de la partie translation.
         * \param double correspondant à la seconde composante de la partie translation en m */
        void y(double _y);

        /** Permet de modifier l'angle de la partie rotation.
         * \param double correspondant à l'angle de la partie rotation en radians */
        void angle(double _heading);

        /** Permet de calculer l'inverse de la Pose.
         * La Pose inverse a un vecteur translation et un angle opposés à ceux initiaux.
         * \remarks l'objet n'est pas modifié.
         * \returns la Pose inversée */
        Pose inverse();

    protected:
        Vector2 positionTranslation;
        Rotation2 positionRotation;

    public:
        /** Opérateur d'affectation.
         * \param _other la Pose qu'on veut dupliquer
         * \returns la Pose initialisée */
        Pose operator =(Pose _other);

        /** Opérateur d'égalité.
         * \param _other la Pose à comparer.
         * \returns vrai si les Poses sont exactement identiques
         * \remarks la comparaison est basée sur la double comparaison des
         * composantes de la partie translation et de l'angle de la partie
         * rotation. La précision utilisée est ici celle des double. */
        bool operator ==(Pose _other);

        //	inline friend Pose operator+(const Pose& lhs, const Pose& rhs);
        //	Pose operator +(Pose other);
        //	Pose operator -(Pose other);
        //	friend std::ostream operator <<(std::ostream os, Pose _pose);
    };

}

#endif /* _ARPMATH_POSE_HPP_ */
