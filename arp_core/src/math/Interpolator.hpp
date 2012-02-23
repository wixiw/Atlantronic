/*
 * Interpolator.hpp
 *
 *  Created on: 23 February 2012
 *      Author: Boris
 */

#ifndef _ARPMATH_INTERPOLATOR_HPP_
#define _ARPMATH_INTERPOLATOR_HPP_

#include <math/math.hpp>

namespace arp_math
{

/**
 *
 * \class Interpolator
 *
 * \brief Interpolator est une classe d'outils qui permettent de faire des interpolations linéaires
 * temporelles sur des vecteurs de position (translation) et d'orientation (angle).
 *
 * Cette classe ne contient que des méthodes statiques. C'est une boîte à outils.
 */
class Interpolator
{
    public:
        /**
         * Permet de réaliser un interpolation linéaire temporelle sur des données en translation.
         * \param tt est un vecteur de temps en seconde de taille P. Il correspond aux points pour lesquels nous
         * voulons connaître la valeur..
         * \param ttc est un vecteur de temps en seconde de taille N. Il correspond aux temps (abscisses) des points de contrôle. Les temps doivent être croissants.
         * \param yyc est un vecteur de valeurs de taille N. Il correspond aux valeurs (ordonnées) des points de contrôle.
         * \returns un vecteur de taille P correspondant aux valeurs des instants tt.
         * \remarks Si ttc et yyc ne sont pas de même taille, transInterp renvoie un vecteur vide.
         * \remarks Si N vaut 0, transInterp renvoie un vecteur vide.
         * \remarks Si N vaut 1, transInterp renvoie un vecteur de taille P rempli de l'unique valeur yyc.
         * \remarks Si des temps de tt sont inférieurs à la première valeur de ttc, transInterp renvoie pour ces temps des valeurs conformes à la première pente des points de contrôle.
         * \remarks Si des temps de tt sont supérieurs à la dernière valeur de ttc, transInterp renvoie pour ces temps des valeurs conformes à la dernière pente des points de contrôle.
         */
        static Eigen::VectorXd transInterp(Eigen::VectorXd tt,
                                           Eigen::VectorXd ttc,
                                           Eigen::VectorXd yyc);

        /**
         * Permet de réaliser un interpolation linéaire temporelle sur des données en rotation.
         * \warning L'écart angulaire entre deux points de contrôle consécutifs ne doit pas être égal exactement à PI, sans quoi la solution n'est pas unique.
         * \param tt est un vecteur de temps en seconde de taille P. Il correspond aux points pour lesquels nous
         * voulons connaître la valeur.
         * \param ttc est un vecteur de temps en seconde de taille N. Il correspond aux temps (abscisses) des points de contrôle. Les temps doivent être croissants.
         * \param hhc est un vecteur d'angles en radians de taille N. Il correspond aux angles (ordonnées) des points de contrôle.\n
         * Les angles de hhc n'ont pas besoin d'être compris entre des bornes particulières.
         * \returns un vecteur de taille P correspondant aux angles des instants tt. Ces angles sont compris entre -PI (exclu) et PI (inclu).
         * \remarks Si ttc et yyc ne sont pas de même taille, transInterp renvoie un vecteur vide.
         * \remarks Si N vaut 0, transInterp renvoie un vecteur vide.
         * \remarks Si N vaut 1, transInterp renvoie un vecteur de taille P rempli de l'unique valeur yyc.
         * \remarks Si des temps de tt sont inférieurs à la première valeur de ttc, transInterp renvoie pour ces temps des valeurs conformes à la première pente des points de contrôle.
         * \remarks Si des temps de tt sont supérieurs à la dernière valeur de ttc, transInterp renvoie pour ces temps des valeurs conformes à la dernière pente des points de contrôle.
         */
        static Eigen::VectorXd rotInterp(Eigen::VectorXd tt,
                                         Eigen::VectorXd ttc,
                                         Eigen::VectorXd hhc);

        /**
         * Permet de trouver la borne inférieure de l'encadrant d'une valeur dans un vecteur.
         * \param t est un temps en seconde.
         * \param ttc est un vecteur de temps en seconde de taille N.
         * \warning Les temps ttc DOIVENT être croissants.
         * \returns l'indice de la borne inférieur encadrant t dans le vecteur ttc.\n
         * Par exemple:\n
         * \li si ttc[k] <= t < ttc[k+1] avec k compris entre 0 et N-2 inclus, find retournera k.
         * \li si ttc[N-1] <= t, find retournera N-1.
         * \li si t < ttc[0], find retournera -1.
         * \remarks Si N vaut 0, find renvoie -1.
         * \remarks Si N vaut 1, find renvoie -1 si t < ttc[0] et 0 si ttc[0] <= t.
         */
        static int find(double t, Eigen::VectorXd ttc);
};

}

#endif /* _ARPMATH_INTERPOLATOR_HPP_ */
