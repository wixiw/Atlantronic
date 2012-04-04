/*
 * Pose2D.hpp
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

#ifndef _ARP_MATH_POSE2D_HPP_
#define _ARP_MATH_POSE2D_HPP_

#include <math/math.hpp>

namespace arp_math
{

/** Matrice 3*3 (Hij) représentant une configuration en 2D */
typedef Eigen::Matrix<double,3,3> Displacement2;

/** Matrice 3*3 des operateurs Grand Adjoints en 2D*/
typedef Eigen::Matrix<double,3,3> BigAdjoint2;

/** \ingroup arp_math
 * \nonstableyet
 *
 * \class Pose2D
 *
 * \brief Pose2D du robot
 *
 * Cette classe représente la pose (position 2D et cap) du robot par rapport à une référence externe.
 * Les unités sont les mètres et les radians.
 */
class Pose2D
{
    public:

        /** Constructeur principal 1.
         * Il permet une initialisation par défaut de la Pose2D à partir d'un Vector2 et d'une Rotation2 */
        Pose2D(Vector2 _translation = Vector2(0., 0.), Rotation2 _orientation = Rotation2(0.));

        /** Constructeur principal 2.
         * Il permet une initialisation par défaut de la Pose2D à partir de 3 double */
        Pose2D(double _x, double _y, double _h = 0.);

        // Getters
        /** \returns la partie translation. Il s'agit d'un Eigen::Vector de taille 2 en m */
        Vector2 translation() const;

        /** \returns la partie rotation de la Pose2D (en radians). */
        Rotation2 orientation() const;

        //Displacement displacement();
        /** \returns la composante x de la partie translation en m */
        double x() const;
        double& xRef();

        /** \returns la composante y de la partie translation en m */
        double y() const;
        double& yRef();

        /** \returns un double image du cap en radians (idem angle() )*/
        double h() const;
        double& hRef();

        /** \returns un double image du cap en radians (idem h() )*/
        double angle() const;

        /** \returns la matrice homogène 3x3. Le bloc haut gauche est constitué de la matrice orthogonale 2x2
         *
         * de la rotation. Le vecteur colonne 2x1 du bloc haut droit est constitué du vecteur translation */
        Displacement2 getDisplacement2Matrix() const;

        //	Eigen::Matrix<double,4,4> matrix4();

        // Setters
        /** Permet de modifier la partie translation.
         * \param _translation Vector2 qui correspond à la partie translation que l'on souhaite en m */
        void translation(Vector2 _translation);

        /** Permet de modifier la partie rotation.
         * \param _orientation Rotation2 qui correspond à la partie rotation que l'on souhaite */
        void orientation(Rotation2 _orientation);

        /** Permet de modifier la première composante de la partie translation.
         * \param double correspondant à la première composante de la partie translation en m */
        void x(double _x);

        /** Permet de modifierARPMath la seconde composante de la partie translation.
         * \param double correspondant à la seconde composante de la partie translation en m */
        void y(double _y);

        /** Permet de modifier l'angle de la partie rotation (idem angle(double) ).
         * \param double correspondant à l'angle de la partie rotation en radians */
        void h(double _heading);

        /** Permet de modifier l'angle de la partie rotation (idem h(double) ).
         * \param double correspondant à l'angle de la partie rotation en radians */
        void angle(double _heading);

        /** Permet de calculer l'inverse de la Pose2D.
         * La Pose2D inverse a un vecteur translation et un angle opposés à ceux initiaux.
         * \remarks l'objet n'est pas modifié.
         * \returns la Pose2D inversée */
        Pose2D inverse() const;

        /**
         * Crée une chaine de caractère représentant la pose sous forme (x,y,theta)
         */
        std::string toString() const;

        /**
         * Construit le grandAdjoint de la pose
         * (1           0)
         * (toHat(p)R   R)
         */
        BigAdjoint2 getBigAdjoint() const;

    protected:
        Vector2 positionTranslation;
        Rotation2 positionRotation;

    public:

        /** Opérateur d'égalité.
         * \param _other la Pose2D à comparer.
         * \returns vrai si les Pose2Ds sont exactement identiques
         * \remarks la comparaison est basée sur la double comparaison des
         * composantes de la partie translation et de l'angle de la partie
         * rotation. La précision utilisée est ici celle des double. */
        bool operator ==(Pose2D _other) const;

        //	inline friend Pose2D operator+(const Pose2D& lhs, const Pose2D& rhs);
        //	Pose2D operator +(Pose2D other);
        //	Pose2D operator -(Pose2D other);
        //	friend std::ostream operator <<(std::ostream os, Pose2D _pose);
};

}

#endif /* _ARP_MATH_POSE2D_HPP_ */
