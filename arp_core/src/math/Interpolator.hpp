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
         * \remarks Si N vaut 1, transInterp renvoie un vecteur de taille P rempli de l'unique valeur de yyc.
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
         * \remarks Si ttc et hhc ne sont pas de même taille, rotInterp renvoie un vecteur vide.
         * \remarks Si N vaut 0, rotInterp renvoie un vecteur vide.
         * \remarks Si N vaut 1, rotInterp renvoie un vecteur de taille P rempli de l'unique valeur de hhc.
         * \remarks Si des temps de tt sont inférieurs à la première valeur de ttc, rotInterp renvoie pour ces temps des valeurs conformes à la première pente des points de contrôle.
         * \remarks Si des temps de tt sont supérieurs à la dernière valeur de ttc, rotInterp renvoie pour ces temps des valeurs conformes à la dernière pente des points de contrôle.
         */
        static Eigen::VectorXd rotInterp(Eigen::VectorXd tt,
                                         Eigen::VectorXd ttc,
                                         Eigen::VectorXd hhc);

        /**
         * Permet de réaliser un interpolation linéaire temporelle sur des covariances.\n
         * Cette interpolation est un peu futtée car elle assure que les covariances résultantes ont bien les propriétés d'une covariance, à savoir qu'elles soient symétriques et strictement positives.\n
         *
         *
         * Une simple interpolation linéaire sur les coefficients ne suffit pas pour assurer que la matrice résultante est une matrice de covariance.\n
         * Pour les termes non diagonaux, Il n'y a pas de pb.\n
         * Pour les termes diagonaux, les termes nuls ne sont pas acceptables pour une matrice de covariance.\n
         * Ainsi, pour ces termes, si l'interpolation linéaire donne une valeur supérieur à epsilon, la valeur est conservée.\n
         * En revanche, si elle donne une valeur inférieure à epsilon, une décroissance exponentielle en \f$y(x) = \exp(\frac{x}{\epsilon-m} + \log(\epsilon-m)-\frac{\epsilon}{\epsilon-m})\f$ est appliquée.\n
         * Cette décroissance assure pour les termes diaguonaux:
         * \li leur stricte positivité
         * \li une borne minimale à \f$m\f$
         * \li leur continuité en \f$x=\epsilon\f$ : \f$y(x) = \epsilon\f$ pour \f$ x \rightarrow \epsilon\f$ par la gauche comme par la droite
         * \li leur dérivabilité et la continuité de leurs dérivées en \f$x=\epsilon\f$: \f$ y'(x) = 1\f$ pour \f$ x \rightarrow \epsilon\f$ par la gauche comme par la droite.\n
         *
         * \param[in] tt est un vecteur de temps en seconde de taille P. Il correspond aux points pour lesquels nous
         * voulons connaître la valeur.
         * \param[in] ttc est un vecteur de temps en seconde de taille N. Il correspond aux temps (abscisses) des covariances de contrôle. Les temps doivent être croissants.
         * \param[in] covc est un vecteur de covariances de taille N. Il correspond aux covariances de contrôle.\n
         * Les covariances doivent être symétriques et positives.
         * \param[in] epsilon Cet epsilon (noté \f$\epsilon\f$) est utilisé pour assurer la stricte positivité des termes diaguonaux. epsilon doit être strictement positif. En dessous de cet epsilon, l'interpolation suit une décroissance exponentielle (voir remarque).
         * \param[in] minimum Ce vecteur (noté \f$m\f$) définit les bornes minimales des trois termes diaguonaux des covariances résultantes. Les coefficients de \f$m\f$ doit être strictement positif et strictement inférieur à \f$\epsilon\f$.
         * \returns un vecteur de covariances de taille P correspondant aux covariances aux instants tt. Ces covariances sont symétriques et positives.
         * \remarks Si ttc et covc ne sont pas de même taille, covInterp renvoie un vecteur vide.
         * \remarks Si N vaut 0, covInterp renvoie un vecteur vide.
         * \remarks Si N vaut 1, covInterp renvoie un vecteur de taille P rempli de l'unique valeur de covc.
         * \remarks Si des temps de tt sont inférieurs à la première valeur de ttc, covInterp renvoie pour ces temps des valeurs conformes à la première pente des points de contrôle.
         * \remarks Si des temps de tt sont supérieurs à la dernière valeur de ttc, covInterp renvoie pour ces temps des valeurs conformes à la dernière pente des points de contrôle.
         */
        static Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covInterp(Eigen::VectorXd tt,
                                                                     Eigen::VectorXd ttc,
                                                                     Eigen::Array< Eigen::Matrix3d, Eigen::Dynamic, 1 > covc,
                                                                     double epsilon = 1e-6,
                                                                     Eigen::Vector3d minimum = Eigen::Vector3d(1e-9,1e-9,1e-9));

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
