/*
 * LaserOnlyLocalizator.hpp
 *
 *  Created on: 6 Mai 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_LASERONLYLOCALIZATOR_HPP_
#define _ARP_RLU_KFL_LASERONLYLOCALIZATOR_HPP_

#include <math/core>

#include <LSL/LaserScan.hpp>
#include <LSL/objects/Circle.hpp>
#include <LSL/objects/DetectedCircle.hpp>
#include <LSL/filters/MedianFilter.hpp>
#include <LSL/filters/PolarCrop.hpp>
#include <LSL/filters/PolarSegment.hpp>
#include <LSL/filters/CircleIdentif.hpp>
#include <LSL/identificators/TrioCircleIdentif.hpp>

namespace arp_rlu
{

namespace kfl
{

/** \ingroup kfl
 *
 * \class LaserOnlyLocalizator
 *
 * \brief LaserOnlyLocalizator est la classe qui permet d'estimer une Pose .
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
class LaserOnlyLocalizator
{
        /** \ingroup kfl
         *
         * \class Params
         *
         * \brief LaserOnlyLocalizator::Params rassemble les paramètres du traitement de scan.
         *
         */
        class Params
        {
            public:
                /** Constructeur par défault.
                 *  Il initialise des paramètres classiques non-stupides :\n
                 *  \li mfp => voir constructeur par défault de lsl::MedianFilter::Params
                 *  \li pcp => voir constructeur par défault de lsl::PolarCrop::Params
                 *  \li psp => voir constructeur par défault de lsl::PolarSegment::Params
                 *  \li cip => voir constructeur par défault de lsl::CircleIdentif::Params
                 *  \li tcp => voir constructeur par défault de lsl::CircleIdentif::Params
                 *  \li minNbPoints = 3
                 */
                Params();

                /**
                 * Permet de formatter les paramètres en un message lisible.
                 */
                std::string getInfo() const;

                /**
                 * Permet de vérifier que les paramètres sont consistants.\n
                 * A savoir :\n
                 * \li chaque sous-groupe de paramètres est consistant.
                 */
                bool checkConsistency() const;

                /**
                 * Paramètres pour la première étape du traitement, à savoir un filtrage médian.
                 */
                lsl::MedianFilter::Params mfp;

                /**
                 * Paramètres pour la deuxième étape du traitement, à savoir un crop polaire qui permet de s'affranchir des mesures trop lointaines.
                 */
                lsl::PolarCrop::Params pcp;

                /**
                 * Paramètres pour la troisième étape, celle qui consiste à identifier des clusters de mesures succeptibles d'être des balises.
                 */
                lsl::PolarSegment::Params psp;

                /**
                 * Paramètres pour la quatrième étape, celle qui consiste à identifier les clusters comme étant des cercles.
                 */
                lsl::CircleIdentif::Params cip;

                /**
                 * Paramètres pour la cinquième étape, celle qui consiste à essayer de trouver le triangle des trois balises parmis les cercles détectés.
                 */
                lsl::TrioCircleIdentif::Params tcp;

                /**
                 * Position des balises (circulaires) sur la table
                 */
                std::vector< lsl::Circle > referencedBeacons;

                /**
                 * Nombre minimal de point pour qu'un DetectedObject soit considéré comme une balise potentielle.
                 */
                unsigned int minNbPoints;

                /**
                 * Position du repère hky dans le ropère de référence du robot
                 */
                arp_math::Pose2D H_hky_robot;
        };

        /**
         * Cette méthode permet de réaliser l'estimation de la position.
         * \param[in] ls le Scan à traiter
         * \return Vrai si le calcul de l'estimation s'est bien passé et Faux sinon.\n
         * Faux peut signifier :
         * \li que le LaserScan est vide
         */
        bool process(lsl::LaserScan ls);

};

}
}

#endif /* _ARP_RLU_KFL_BEACONDETECTOR_HPP_ */
