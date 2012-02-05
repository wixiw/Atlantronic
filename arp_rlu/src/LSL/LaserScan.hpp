/*
 * LaserScan.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_LASERSCAN_HPP_
#define _ARP_RLU_LSL_LASERSCAN_HPP_

#include <math/core>

namespace arp_rlu
{

namespace lsl
{

/** \ingroup lsl
 * \nonstableyet
 *
 * \class LaserScan
 *
 * \brief LaserScan est la classe qui représente un scan laser (1.5D).
 *
 * Un scan peut être par exemple issu d'une mesure d'un LRF (Hokuyo etc...)
 * Il correspond à un ensemble de point dans un plan polaire. Sa dimension est donc de 1.5D.\n
 * Un LaserScan est donc d'abord définit dans un repère polaire. il est ensuite possible de calculer
 * les coordonnées des points dans le repère cartésien associé. \n
 *
 * Ce plan polaire (et son repère cartésien associé) est lui même définit par rapport à un repère cartésien de référence.
 */
class LaserScan
{
    public:
        /** Constructeur par défault.
         *  Il construit un scan vide.
         */
        LaserScan();

        /** Constructeur par copy.
         */
        LaserScan(const LaserScan &);

        /** Permet d'obtenir la taille du scan, c'est à dire le nombre de points qui le compose.
         * \returns la taille du scan
         **/
        unsigned int getSize() const;

        /** Calcule les coordonnées cartésiennes à partir des données polaires et de la position
         * du repère polaire par rapport au repère de référence.
         * \param tt un vecteur de taille P. Il contient des temps en secondes. C'est temps doivent être croissants.
         * \param xx un vecteur de taille P. Il contient les positions selon x de l'origine du repère polaire dans le repère de référence.
         * \param yy un vecteur de taille P. Il contient les positions selon y de l'origine du repère polaire dans le repère de référence.
         * \param hh un vecteur de taille P. Il contient les orientations du repère polaire par rapport au repère de référence.
         * \returns Vrai si le calcul c'est bien passé et Faux sinon
         * \remarks Les 4 vecteurs doivent impérativement être de même taille.
         * Ils correspondent à des poses du repère polaire par rapport au repère cartésien.*/
        bool computeCartesianData(Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh);


        /** Permet de modifier les données polaires du scan.
         * \param data Une matrice de taille (3,N) avec N le nombre de points dans le scan.\n
         * La première ligne correspond à la date en seconde d'acquisition des points.\n
         * La deuxième ligne correspond à r, la distance du point à l'origine du repère polaire.\n
         * La troisième ligne correspond à theta, l'angle en radian du vecteur (origine -> point)
         * par rapport à l'axe de référence du repère polaire.\n
         * \remarks Les données cartésiennes qui auraient été précédemment calculées ne sont plus disponibles.\n*/
        void setPolarData(Eigen::MatrixXd data);

        /** Permet d'accéder aux coodonnées polaires des points du scan
         * \returns Une matrice de taille (3,N) avec N le nombre de points dans le scan.\n
         * La première ligne correspond à la date en seconde d'acquisition des points.\n
         * La deuxième ligne correspond à r, la distance du point à l'origine du repère polaire.\n
         * La troisième ligne correspond à theta, l'angle en radian du vecteur (origine -> point)
         * par rapport à l'axe de référence du repère polaire.\n
         * \remarks Si le scan est vide, la matrice renvoyée est de taille (3,0)*/
        Eigen::MatrixXd getPolarData() const;

        /** Permet d'accéder aux coordonnées cartésiennes des points du scan.
         * \returns Une matrice de taille (3,N) avec N le nombre de points dans le scan.\n
         * La première ligne correspond à la date en seconde d'acquisition des points. \n
         * La deuxième ligne correspond aux coordonnées selon l'axe x du repère de référence. \n
         * La troisième ligne correspond aux coordonnées selon l'axe y du repère de référence. \n
         * \remarks Si le scan est vide, la matrice renvoyée est de taille (3,0)*/
        Eigen::MatrixXd getCartesianData() const;

        /** Permet d'accéder aux coordonnées cartésiennes des points du scan.
         * \returns Un vecteur de taille N avec N le nombre de points dans le scan.\n
         * Il correspond à la date d'acquisition en seconde des points du scan. */
        Eigen::VectorXd getTimeData() const;

        /** Permet de savoir si les données cartésiennes sont disponibles, c'est à dire si elles ont été calculées.
         * \returns Vrai si les données sont disponibles et Faux sinon.*/
        bool areCartesianDataAvailable();

        /** Permet de retirer les points de rayon nul.
         * \returns Vrai s'il y avait effectivement quelque chose à nettoyer.
         */
        bool cleanUp();

    protected:
        Eigen::MatrixXd data;

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_LASERSCAN_HPP_ */
