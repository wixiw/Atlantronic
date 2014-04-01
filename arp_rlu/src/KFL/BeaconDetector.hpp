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

#include <LSL/LaserScan.hpp>
#include <LSL/objects/Circle.hpp>
#include <LSL/objects/DetectedCircle.hpp>
#include <LSL/filters/MedianFilter.hpp>
#include <LSL/filters/PolarCrop.hpp>
#include <LSL/filters/PolarSegment.hpp>
#include <LSL/filters/CircleIdentif.hpp>
#include <LSL/identificators/TrioCircleIdentif.hpp>
#include <LSL/identificators/DuoCircleIdentif.hpp>

#include <time/StatTimer.hpp>

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
             *  \li mfp => voir constructeur par défault de lsl::MedianFilter::Params
             *  \li pcp => voir constructeur par défault de lsl::PolarCrop::Params
             *  \li psp => voir constructeur par défault de lsl::PolarSegment::Params
             *  \li cip => voir constructeur par défault de lsl::CircleIdentif::Params
             *  \li tcp => voir constructeur par défault de lsl::CircleIdentif::Params
             *  \li dcp => voir constructeur par défault de lsl::CircleIdentif::Params
             *  \li minNbPoints = 3
             *  \li xMin = -2.0
             *  \li xMax =  2.0
             *  \li yMin = -1.5
             *  \li yMax =  1.5
             *  \li xMinObstacle = -1.2
             *  \li xMaxObstacle =  1.2
             *  \li yMinObstacle = -1.0
             *  \li yMaxObstacle =  1.0
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
             * Minimum cartésien selon X pour la recherche de balise
             */
            double xMin;

            /**
             * Maximum cartésien selon X pour la recherche de balise
             */
            double xMax;

            /**
             * Minimum cartésien selon Y pour la recherche de balise
             */
            double yMin;

            /**
             * Maximum cartésien selon Y pour la recherche de balise
             */
            double yMax;

            /**
             * Minimum cartésien selon X pour la détection d'obstacle
             */
            double xMinObstacle;

            /**
             * Maximum cartésien selon X pour la détection d'obstacle
             */
            double xMaxObstacle;

            /**
             * Minimum cartésien selon Y pour la détection d'obstacle
             */
            double yMinObstacle;

            /**
             * Maximum cartésien selon Y pour la détection d'obstacle
             */
            double yMaxObstacle;

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
         * Permet pour une date donnée de savoir si une balise est vue et de connaitre laquelle ainsi que sa mesure.
         * \param[in] t la date à considérer.
         * \param[out] target indique la balise qui est observée.
         * \param[out] meas indique la mesure (en polaire) de la balise observée.\n
         * Le premier élément correspond au range r, le second à l'angle theta.
         */
        bool getBeacon(double t, lsl::Circle & target, Eigen::Vector2d & meas);

        /**
         * Permet de récupérer tous les clusters candidats.
         * \remark Cette méthode est présente pour le débug seulement.\n
         */
        std::vector< lsl::DetectedObject > getDetectedObjects();

        /**
         * Permet de récupérer tous les cercles candidats.
         * \remark Cette méthode est présente pour le débug seulement.\n
         */
        std::vector< lsl::DetectedCircle > getDetectedCircles();

        /**
         * Permet de récupérer les balises détectées.
         * \remark Cette méthode est surtout pratique pour le débug.\n
         * Elle évite d'avoir à appelée N fois la méthode \ref getBeacon
         */
        std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > getFoundBeacons();

        /**
         * Permet de récupérer les obstacles détectés sur la table.
         */
        std::vector< arp_math::Vector2 > getDetectedObstacles();

        /**
         * Permet de modifier les paramètres de traitement de scan.
         * \param[in] paramètres sous la forme d'un kfl::BeaconDetector::Params
         */
        void setParams(kfl::BeaconDetector::Params);

        /**
         * Permet d'obtenir un rapport sur les timings
         */
        std::string getPerformanceReport();

    protected:
        /**
         * Remise à zéro du traitement.\n
         * Cette méthode réinitialise les résultats.
         */
        void reset();

        /**
         * Paramètres du BeaconDetector
         */
        BeaconDetector::Params params;

        /**
         * Les clusters candidats.\n
         * Ils n'ont pas (encore) été identifiés comme des cercles.
         */
        std::vector< lsl::DetectedObject > detectedObjects;

        /**
         * Les cercles détectés dans la zone cartésienne définies par xMin, xMax, yMin et yMax.
         */
        std::vector< lsl::DetectedCircle > detectedCircles;

        /**
         * Les obstacles détectés sur le terrain.\n
         * Les bornes cartésiennes de la zone de détection sont définies par xMinObstacle, xMaxObstacle, yMinOBstacle et yMaxObstacle
         */
        std::vector< arp_math::Vector2 > detectedObstacles;

        /**
         * Les obstacles détectés sur le terrain.\n
         * Les bornes cartésiennes de la zone de détection sont définies par xMinObstacle, xMaxObstacle, yMinOBstacle et yMaxObstacle
         */
        std::vector< lsl::DetectedCircle > beaconCandidates;

        /**
         * Balises retrouvées
         */
        std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > foundBeacons;

        /**
         * Balises référencées
         */
        std::vector<lsl::Circle> referencedBeacons;

        /**
         * Temps mini entre deux mesures du scan
         */
        double deltaTime;

        arp_core::StatTimer mfTimer;
        arp_core::StatTimer pcTimer;
        arp_core::StatTimer clTimer;
        arp_core::StatTimer cartTimer;
        arp_core::StatTimer psTimer;
        arp_core::StatTimer fiTimer;
        arp_core::StatTimer ciTimer;
        arp_core::StatTimer obsTimer;
        arp_core::StatTimer tcTimer;
        arp_core::StatTimer dcTimer;
        arp_core::StatTimer globalTimer;

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BEACONDETECTOR_HPP_ */
