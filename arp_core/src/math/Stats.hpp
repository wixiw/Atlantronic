/*
 * Stats.hpp
 *
 *  Created on: 23 February. 2012
 *      Author: Boris
 */

#ifndef _ARPMATH_STATS_HPP_
#define _ARPMATH_STATS_HPP_

#include <math/math.hpp>

namespace arp_math
{
    /**
     * Calcule la moyenne d'un vecteur Eigen
     * \params le vecteur que l'on souhaite moyenner
     * \returns la moyenne
     * \remarks renvoie 0 sur le vecteur est vide.
     */
    double mean(const Eigen::VectorXd & v);

    /**
     * Calcule l'écart type d'un vecteur Eigen
     * \params le vecteur que l'on souhaite considérer
     * \returns l'écart type
     * \remarks renvoie 0 sur le vecteur est vide.
     */
    double stddev(const Eigen::VectorXd & v);

    /**
     * Calcule la median d'un vecteur Eigen
     * \params le vecteur que l'on souhaite considérer
     * \returns la mediane
     * \remarks renvoie 0 sur le vecteur est vide.
     * \remarks si le vecteur est de taille paire, il renvoie la mediane inférieure.
     */
    double median(const Eigen::VectorXd & v);
}


#endif /* _ARPMATH_STATS_HPP_ */
