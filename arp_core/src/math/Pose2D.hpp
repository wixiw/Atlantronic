/*
 * Pose2D.hpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

// TODO BMO :
// * utiliser les références et les const
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

        /** Constructeur principal.
         * Il permet une initialisation par défaut de la Pose2D à partir de 3 double */
        Pose2D(double _x = 0., double _y = 0., double _h = 0.);

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

        /**
         * Retourne la pose sous forme de Vecteur 3 (x,y,theta)
         */
        Vector3 getTVector() const;

        /**
         * \returns la matrice symétrique 2x2 correspondant à la partie rotation.\n
         * Il s'agit d'un racourcis de this->orientation().toMatrixRotation()
         */
        Eigen::Matrix<double,2,2> getRotationMatrix() const;

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
         * \returns la Pose2D inversée
         * \warning l'inverse n'est pas l'opposé ! Il s'agit de l'inverse tel que la Pose2D composée avec son inverse donne la Pose2D nulle. */
        Pose2D inverse() const;

        /**
         * Permet de prendre l'opposé de la pose défini par l'opposé de la translation et l'opposé de la rotation
         */
        Pose2D opposite() const;

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

        /**
         * Calcule la distance entre 2 positions (en 2D, sans tenir compte de l'angle)
         */
        double distanceTo(Pose2D pose) const;

        /**
         * Calcule l'angle entre 2 positions
         * comptée positive de notre angle vers celui de la pose en paramètre
         * retour normalisé entre -Pi et Pi
         */
        double angleTo(Pose2D pose) const;

        /**
         * calcule la norme du vecteur
         */
        double vectNorm() const;
        /**
         * calcule l'angle du vecteur
         */
        double vectAngle() const;

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

        /**
         * Les Pose2D évoluent dans le groupe spécial euclidien nommé SE(2)\n
         * L'opérateur * représente la loi de composition interne de ce groupe.\n
         * Elle est très pratique pour réaliser des changements de repère.\n
         * Ex :
         * \li soit H_1_0 une Pose2D correspondant à la position d'un repère 1 par rapport à un repère 0
         * \li soit H_2_1 une Pose2D correspondant à la position d'un repère 2 par rapport au repère 1
         * \li soit H_2_0 une Pose2D correspondant à la position du repère 2 par rapport au repère 0
         * \li soit H_0_2 une Pose2D correspondant à la position du repère 0 par rapport au repère 2
         * Alors :
         * \li H_2_0 = H_1_0 * H_2_1;
         * \li H_0_2 = H_2_0.inverse();
         * \li H_0_2 = H_2_1.inverse() * H_1_0.inverse();
         */
        Pose2D operator*(const Pose2D& other) const;

        /**
         * Permet de changer un point de repère de référence. \n
         * Ex :
         * \li soit v1 un Vector2 exprimé dans un repère 1
         * \li soit H_1_0 une Pose2D correspondant à la position du repère 1 par rapport à un repère 0
         * \li soit v2 le vecteur v1 exprimé dans le repère 0
         * Alors :
         * \li v2 = H_1_0 * v1;
         * \li v1 = H_1_0.inverse() * v2;
         * \warning cette méthode s'applique sur un point, pas sur un vecteur. La nuance vient qu'on prend pas en compte seulement
         * les orientatations des deux repères, mais aussi le déport de leurs origines.
         */
        Vector2 operator*(const Vector2& v) const;
};

}

std::ostream operator <<(std::ostream os, arp_math::Pose2D _pose);
arp_math::Pose2D operator*(const double scalaire, const arp_math::Pose2D& pose);

#endif /* _ARP_MATH_POSE2D_HPP_ */
