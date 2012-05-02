/*
 * MathFactory.hpp
 *
 *  Created on: 2 Mai 2012
 *      Author: Boris
 */

#ifndef _ARP_MATH_MATHFACTORY_HPP_
#define _ARP_MATH_MATHFACTORY_HPP_

#include <math/math.hpp>
#include <math/Pose2D.hpp>
#include <math/EstimatedPose2D.hpp>
#include <math/Twist2D.hpp>
#include <math/EstimatedTwist2D.hpp>

namespace arp_math
{
class MathFactory
{
    public:

        //----------------------------------------------------------------------------------
        //----------------------  Pose -----------------------------------------------------
        //----------------------------------------------------------------------------------

        /** Permet de construire une Pose à partir d'un Vector2 et d'une Rotation2
         * @param[in] translation est un vecteur de taille correspondant à la partie translation
         * @param[in] orientation est une Rotation2 correspondant à la partie rotation
         **/
        static Pose2D createPose2D(Vector2 translation, Rotation2 orientation);



        //----------------------------------------------------------------------------------
        //----------------------  Pose -----------------------------------------------------
        //----------------------------------------------------------------------------------

        /** Permet de construire une EstimatedPose à partir d'un Vector2, d'une Rotation2, d'une date et d'une covariance
         * @param[in] translation est un vecteur de taille correspondant à la partie translation
         * @param[in] orientation est une Rotation2 correspondant à la partie rotation
         * @param[in] date est un double correspondant à la date d'estimation
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         **/
        static EstimatedPose2D createEstimatedPose2D(Vector2 translation, Rotation2 orientation, long double date, Covariance3 cov );

        /** Permet de construire une EstimatedPose à partir de trois double, d'une date et d'une covariance
         * @param[in] x est la translation selon l'axe x
         * @param[in] y est la translation selon l'axe y
         * @param[in] h est la rotation autour de l'axe z
         * @param[in] date est un double correspondant à la date d'estimation
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         **/
        static EstimatedPose2D createEstimatedPose2D(double x, double y, double h, long double date, Covariance3 cov );


        //----------------------------------------------------------------------------------
        //----------------------  Twist ----------------------------------------------------
        //----------------------------------------------------------------------------------

        /**
         * Permet de construire un Twist à partir de sa représentation cartésienne
         * @param[in] vx est un double correspondant à la vitesse de translation selon l'axe x
         * @param[in] vy est un double correspondant à la vitesse de translation selon l'axe y
         * @param[in] vh est un double correspondant à la omega
         *
         */
        static Twist2D createTwist2DFromCartesianRepr(double vx, double vy, double vh = 0);

        /**
         * Permet de construire un Twist à partir de sa représentation cartésienne
         * @param[in] vitesseTranslation est un vecteur de taille correspondant à la vitesse de translation (vx, vy)
         * @param[in] vitesseRotation est un double correspondant à la omega
         */
        static Twist2D createTwist2DFromCartesianRepr(Vector2 vitesseTranslation, double vitesseRotation);

        /**
         * Permet de construire un Twist à partir de sa représentation vectorielle
         * @param[in] T est un vecteur de taille correspondant correspondant à (omega, vx, vy)
         */
        static Twist2D createTwist2DFromCartesianRepr(Vector3 T);


        /**
         * Permet de construire un Twist à partir de sa représentation polaire sous forme d'un triplet de double
         * @param[in] normV est la norme de la vitesse linéaire
         * @param[in] angV est l'angle de la vitesse linéaire
         * @param[in] vh est la norme de la vitesse de rotation
         */
        static Twist2D createTwist2DFromPolarRepr(double normV, double angV, double vh);



        //----------------------------------------------------------------------------------
        //----------------------  EstimatedTwist -------------------------------------------
        //----------------------------------------------------------------------------------

        /**
         * Permet de construire un EstimatedTwist à partir de sa représentation cartésienne en triplet de double
         * @param[in] vx est un double correspondant à la vitesse de translation selon l'axe x
         * @param[in] vy est un double correspondant à la vitesse de translation selon l'axe y
         * @param[in] vh est un double correspondant à la omega
         * @param[in] date est un double correspondant à la date d'estimation du twist
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         *
         */
        static EstimatedTwist2D createEstimatedTwist2DFromCartesianRepr(double vx, double vy, double vh, long double date, Covariance3 cov);

        /**
         * Permet de construire un EstimatedTwist à partir de sa représentation cartésienne en Vector2 et Rotation2
         * @param[in] vitesseTranslation est un vecteur de taille correspondant à la vitesse de translation (vx, vy)
         * @param[in] vitesseRotation est un double correspondant à la omega
         * @param[in] date est un double correspondant à la date d'estimation du twist
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         *
         */
        static EstimatedTwist2D createEstimatedTwist2DFromCartesianRepr(Vector2 vitesseTranslation, double vitesseRotation, long double date, Covariance3 cov);

        /**
         * Permet de construire un EstimatedTwist à partir de sa représentation vectorielle
         * @param[in] T est un vecteur de taille correspondant correspondant à (omega, vx, vy)
         * @param[in] date est un double correspondant à la date d'estimation du twist
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         *
         */
        static EstimatedTwist2D createEstimatedTwist2DFromCartesianRepr(Vector3 T, long double date, Covariance3 cov);


        /**
         * Permet de construire un EstimatedTwist à partir de sa représentation polaire sous forme d'un triplet de double
         * @param[in] normV est la norme de la vitesse linéaire
         * @param[in] angV est l'angle de la vitesse linéaire
         * @param[in] vh est la norme de la vitesse de rotation
         * @param[in] date est un double correspondant à la date d'estimation du twist
         * @param[in] cov est une matrice 3x3 correspondant à la matrice de covariance de l'estimation
         */
        static EstimatedTwist2D createEstimatedTwist2DFromPolarRepr(double normV, double angV, double vh, long double date, Covariance3 cov);
};
}

#endif // _ARP_MATH_MATHFACTORY_HPP_
