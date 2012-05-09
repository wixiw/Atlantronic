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
#include <LSL/identificators/DuoCircleIdentif.hpp>

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
    public:
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
             *  \li cartSigmaMax = 0.05
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
             * Paramètres pour la sixième étape, celle qui consiste à essayer de trouver un segment de deux balises parmis les cercles détectés.
             */
            lsl::DuoCircleIdentif::Params dcp;

            /**
             * Nombre minimal de point pour qu'un DetectedObject soit considéré comme une balise potentielle.
             */
            unsigned int minNbPoints;

            /**
             * Ecart type max des mesures d'un DetectObject pour le considérer comme balise potentielle
             */
            double cartStddevMax;

            /**
             * Définit la zone accessible sur la table
             */
            double xMinAccessible;

            /**
             * Définit la zone accessible sur la table
             */
            double xMaxAccessible;

            /**
             * Définit la zone accessible sur la table
             */
            double yMinAccessible;

            /**
             * Définit la zone accessible sur la table
             */
            double yMaxAccessible;

            /**
             * Position du repère hky dans le ropère de référence du robot
             */
            arp_math::Pose2D H_hky_robot;

            /**
             * Balises référencées
             */
            std::vector<lsl::Circle> referencedBeacons;
        };

    public:

        /**
         * Cstor
         */
        LaserOnlyLocalizator();

        /**
         * Cette méthode permet de réaliser l'estimation de la position.
         * \param[in] ls le Scan à traiter
         * \return Vrai si le calcul de l'estimation s'est bien passé et Faux sinon.\n
         * Faux peut signifier :
         * \li que le LaserScan est vide
         * \li qu'il y a moins de 3 balises référencées
         * \li qu'il n'a pas réussi le calcul (pas une bonne visibilité)
         */
        bool process(lsl::LaserScan ls);

        /**
         * Permet de modifier les paramètres.
         * \param[in] paramètres sous la forme d'un kfl::LaserOnlyLocalizator::Params
         */
        void setParams(kfl::LaserOnlyLocalizator::Params);

        /**
         *  Permet d'accéder à l'estimée calculée si le calcul a réussi.\n
         *  Il s'agit de H_hky_table
         */
        arp_math::EstimatedPose2D getEstimatedPose();

        /**
         *  Conseille un cap relatif à effecter afin de confirmer/améliorer le résultat.
         *  \li Si le calcul a réussi, le cap permet de placer le robot afin qu'il puisse voir 3 balises.
         *  \li Si le calcul a échoué, le cap est de +60°.
         */
        arp_math::Rotation2 getRelativeHeadingForConfirmation();


        /**
         * Permet d'obtenir un rapport sur les timings
         */
        std::string getPerformanceReport();

    public:
        /**
         * Permet d'estimer une position à partir de l'observation d'une constellation connue.
         * \param[in] vecteur de paire d'appariement mesures points d'intéret <=> position de référence
         * \param[out] l'estimée de position sir le calcul s'est bien passé
         * \return un booléen de réussite
         * \li si le vpds est vide, retourne false et pose n'est pas modifiée
         * \li si le vpds est de taille 1, retourne false et pose n'est pas modifiée
         * \li si le vpds est de taille 2, retourne true et modifie l'estimée
         * \li si l'estimée est hors de la zone accessible de la table (cf paramètres), retourne false et pose n'est pas modifiée
         */
        bool estimateFromConstellation(const std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > & vpdc, arp_math::EstimatedPose2D & pose );

    protected:
        /**
         * Paramètres du LaserOnlyLocalizator
         */
        LaserOnlyLocalizator::Params params;

        /**
         * Dernière estimée
         */
        arp_math::EstimatedPose2D lastEstimate;

        /**
         * Dernier cap relatif de confirmation
         */
        arp_math::Rotation2 lastRotation;

        /**
         * Les clusters candidats.\n
         * Ils n'ont pas (encore) été identifiés comme des cercles.
         */
        std::vector< lsl::DetectedObject > detectedObjects;

        /**
         * Les cercles détectés dans la zone cartésienne définies par xMin, xMax, yMin et yMax.
         */
        std::vector< lsl::DetectedCircle > detectedCircles;

        arp_core::StatTimer mfTimer;
        arp_core::StatTimer pcTimer;
        arp_core::StatTimer clTimer;
        arp_core::StatTimer cartTimer;
        arp_core::StatTimer psTimer;
        arp_core::StatTimer fiTimer;
        arp_core::StatTimer ciTimer;
        arp_core::StatTimer tcTimer;
        arp_core::StatTimer dcTimer;
        arp_core::StatTimer globalTimer;


};

}
}

#endif /* _ARP_RLU_KFL_BEACONDETECTOR_HPP_ */
