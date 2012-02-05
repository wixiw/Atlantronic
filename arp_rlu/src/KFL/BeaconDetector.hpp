/*
 * BeaconDetector.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BEACONDETECTOR_HPP_
#define _ARP_RLU_KFL_BEACONDETECTOR_HPP_

#include <vector>
#include <utility>

#include <math/core>

#include "LSL/LaserScan.hpp"
#include "LSL/objects/Circle.hpp"
#include "LSL/filters/MedianFilter.hpp"
#include "LSL/filters/PolarCrop.hpp"
#include "LSL/filters/CartesianCrop.hpp"
#include "LSL/filters/PolarSegment.hpp"



namespace arp_rlu
{

namespace kfl
{

/** \ingroup kfl
 * \nonstableyet
 *
 * \class BeaconDetector
 *
 * \brief BeaconDetector est la classe qui concentre les traitements de scan pour KFL.
 *
 * C'est cette classe qui utilise BFL pour extraire la mesure des balises.
 */
class BeaconDetector
{
    public:
        /** \ingroup kfl
         * \nonstableyet
         *
         * \class Params
         *
         * \brief BeaconDetector::Params rassemble les paramètres du traitement de scan.
         *
         */
        class Params
        {
        public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  mfp => voir contructeur par défault de lsl::MedianFilter::Params \n
             *  pcp => voir contructeur par défault de lsl::PolarCrop::Params \n
             *  ccp => voir contructeur par défault de lsl::CartesianCrop::Params \n
             *  psp => voir contructeur par défault de lsl::PolarSegment::Params \n
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Paramètres pour la première étape du traitement, à savoir un filtrage médian.
             */
            lsl::MedianFilter::Params mfp;

            /**
             * Paramètres pour la deuxième étape du traitement, à savoir un crop polaire qui permet de s'affranchir des mesures trop lointaines.
             */
            lsl::PolarCrop::Params pcp;

            /**
             * Paramètres pour la troisème étape du traitement, à savoir un crop cartésien qui permet d'éliminer les mesures qui sont assurément hors-table.
             */
            lsl::CartesianCrop::Params ccp;

            /**
             * Paramètres pour la quatrième étape, celle qui consiste à identifier les balises potentielles dans le scan.
             */
            lsl::PolarSegment::Params psp;
        };

    public:
        /**
         * Constructeur par défault qui initialise les paramètres avec les valeurs par défault.
         */
        BeaconDetector();

        /**
         * Destructeur par défault
         */
        ~BeaconDetector();

        /**
         * Cette méthode
         * \param ls le Scan à traiter
         * \param tt un vecteur de taille P. Il contient des temps en secondes. C'est temps doivent être croissants.
         * \param xx un vecteur de taille P. Il contient les positions selon x de l'origine du scan dans le repère de référence.
         * \param yy un vecteur de taille P. Il contient les positions selon y de l'origine du scan dans le repère de référence.
         * \param hh un vecteur de taille P. Il contient les orientations du repère du scan par rapport au repère de référence.
         * \remarks Les 4 vecteurs doivent impérativement être de même taille.
         * Ils correspondent à des poses du repère du scan par rapport au repère cartésien de référence.\n
         * Ils servent à redresser le scan quand le LRF a bougé pendant l'acquisition.
         * \return Vrai si le traitement s'est bien passé et Faux sinon.
         */
        bool process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh);

        /**
         * \param beacons un vecteur de cercles qui correspond aux balises référencées.
         */
        void setRefecencedBeacons(std::vector<lsl::Circle> beacons);

        /**
         * Permet pour une date donnée de savoir si une balise est vue et de connaitre la quelle ainsi que sa mesure.
         * \param t la date à considérer.
         * \param target est un paramètre de sortie qui indique la balise qui est observée.
         * \param meas est un paramètre de sortie qui indique la mesure (en polaire) de la balise observée.\n
         * Le premier élément correspond au range r, le second à l'angle theta.
         */
        bool getBeacon(double t, lsl::Circle & target, Eigen::Vector2d & meas);

        /**
         * Permet de modifier les paramètres de traitement de scan.
         * \param paramètres sous la forme d'un kfl::BeaconDetector::Params
         */
        void setParams(kfl::BeaconDetector::Params);

    protected:
        /**
         * Remise à zéro du traitement.\n
         * Cette méthode réinitialise les résultats.
         */
        void reset();

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BEACONDETECTOR_HPP_ */
