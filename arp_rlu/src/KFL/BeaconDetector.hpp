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
 *
 * \class BeaconDetector
 *
 * \brief BeaconDetector est la classe qui concentre les traitements de scan pour KFL.
 *
 * C'est cette classe qui utilise BFL pour extraire la mesure des balises.
 *
 *  Le cycle de vie standard est :
 *  -# construction du BeaconDetector
 *  -# choix des paramètres de traitement de scan via la méthode \ref setParams
 *  -# référencement des balises présentes sur le terrain via la méthode \ref setReferencedBeacons
 *  -# traitement d'un scan via la méthode \ref process
 *  -# récupération des résultats via de multiples appels à la méthode \ref getBeacon
 *  -# nouveau traitement de scan via la méthode \ref process etc...
 *  -# destruction du BeaconDetector
 */
class BeaconDetector
{
    public:
        /** \ingroup kfl
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
             *  mfp => voir constructeur par défault de lsl::MedianFilter::Params \n
             *  pcp => voir constructeur par défault de lsl::PolarCrop::Params \n
             *  ccp => voir constructeur par défault de lsl::CartesianCrop::Params \n
             *  psp => voir constructeur par défault de lsl::PolarSegment::Params \n
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
         * Constructeur par défault qui initialise les paramètres avec les valeurs par défault non-stupides.
         */
        BeaconDetector();

        /**
         * Destructeur par défault
         */
        ~BeaconDetector();

        /**
         * Cette méthode permet de réaliser le traitement du scan.
         * \param[in] ls le Scan à traiter
         * \param[in] tt un vecteur de taille P. Il contient des temps en secondes. Ces temps DOIVENT être croissants.
         * \param[in] xx un vecteur de taille P. Il contient les positions selon l'axe x de l'origine du scan dans le repère de référence.
         * \param[in] yy un vecteur de taille P. Il contient les positions selon l'axe y de l'origine du scan dans le repère de référence.
         * \param[in] hh un vecteur de taille P. Il contient les orientations du repère du scan par rapport au repère de référence.
         * \pre Les 4 vecteurs tt, xx, yy et hh doivent impérativement être de même taille.
         * \remarks Les 4 vecteurs correspondent à des poses du repère du scan par rapport au repère cartésien de référence.\n
         * Ils servent à redresser le scan quand le LRF a bougé pendant l'acquisition.
         * \return Vrai si le traitement s'est bien passé et Faux sinon.
         */
        bool process(lsl::LaserScan ls, Eigen::VectorXd tt, Eigen::VectorXd xx, Eigen::VectorXd yy, Eigen::VectorXd hh);

        /**
         * \param beacons un vecteur de cercles qui correspond aux balises référencées.
         */
        void setReferencedBeacons(std::vector<lsl::Circle> beacons);

        /**
         * Permet pour une date donnée de savoir si une balise est vue et de connaitre la quelle ainsi que sa mesure.
         * \param[in] t la date à considérer.
         * \param[out] target indique la balise qui est observée.
         * \param[out] meas indique la mesure (en polaire) de la balise observée.\n
         * Le premier élément correspond au range r, le second à l'angle theta.
         */
        bool getBeacon(double t, lsl::Circle & target, Eigen::Vector2d & meas);

        /**
         * Permet de modifier les paramètres de traitement de scan.
         * \param[in] paramètres sous la forme d'un kfl::BeaconDetector::Params
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
