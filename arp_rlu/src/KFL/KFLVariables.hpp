/*
 * KFLVariables.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLVARIABLES_HPP_
#define _ARP_RLU_KFL_KFLVARIABLES_HPP_

#include <math/core>

namespace arp_rlu
{

namespace kfl
{

/*!
 *  \addtogroup kfl
 *  @{
 */

/**
 *
 * \struct KFLStateVar
 *
 * \brief KFLStateVar est le type de variable d'état de KFL.
 *
 * Il s'agit d'un vecteur 3 :\n
 * La première coordonnée est la position selon l'axe X (en m).\n
 * La deuxième coordonnée est la position selon l'axe Y (en m).\n
 * La troisième coordonnée est l'angle autour de l'axe Z (en rad).\n
 */
typedef Eigen::Vector3d           KFLStateVar;

/**
 *
 * \struct KFLStateCov
 *
 * \brief KFLStateCov est la covariance associée à la variable d'état KFLStateVar.
 *
 * Il s'agit d'une matrice 3x3
 */
typedef Eigen::Matrix<double,3,3> KFLStateCov;


/**
 *
 * \struct KFLInputVar
 *
 * \brief KFLInputVar est le type de variable d'entrée de KFL.
 *
 * Il s'agit d'un vecteur 3 :\n
 * La première coordonnée est la vitesse linéaire selon l'axe X (en m/s).\n
 * La deuxième coordonnée est la vitesse linéaire selon l'axe Y (en m/s).\n
 * La troisième coordonnée est la vitesse de rotation autour de l'axe Z (en rad/s).\n
 */
typedef Eigen::Vector3d           KFLInputVar;


/**
 *
 * \struct KFLInputCov
 *
 * \brief KFLInputCov est la covariance associée à la variable d'entrée KFLSysInput.
 *
 * Il s'agit d'une matrice 3x3
 */
typedef Eigen::Matrix<double,3,3> KFLInputCov;


/**
 *
 * \struct KFLMeasVar
 *
 * \brief KFLMeasVar est le type de variable de mesure de KFL.
 *
 * Il s'agit d'un vecteur 2 :\n
 * La première coordonnée est la distance (range) en polaire en m.\n
 * La seconde coordonnée est l'angle (theta) en polaire en rad.\n
 */
typedef Eigen::Vector2d           KFLMeasVar;


/**
 *
 * \struct KFLMeasTarget
 *
 * \brief KFLMeasTarget est le type de cible pour KFL : un point du plan que la mesure observe (balise).
 *
 * Il s'agit d'un vecteur 2 :\n
 * La première coordonnée est la position de la cible selon l'axe X (en m).\n
 * La seconde coordonnée est la position de la cible selon l'axe Y (en m).\n
 */
typedef Eigen::Vector2d           KFLMeasTarget;



/*! @} End of Doxygen Groups*/

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLVARIABLES_HPP_ */
