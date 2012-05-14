/*
 * KFLocalizator.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLOCALIZATOR_HPP_
#define _ARP_RLU_KFL_KFLOCALIZATOR_HPP_

#include <math/core>

#include <KFL/BeaconDetector.hpp>
#include <KFL/BayesianWrapper.hpp>

#include <boost/circular_buffer.hpp>

#include <timer/StatTimer.hpp>

namespace arp_rlu
{

namespace kfl
{
/** \ingroup kfl
 *
 * \class KFLocalizator
 *
 * \brief KFLocalizator est l'unique classe qui permet d'accéder depuis l'extérieur de KFL aux fonctionnalités de localisation par fusion odo/laser.
 *
 * Cette classe rassemble la totalité des méthodes permettant de localiser un robot mobile en exploitant des données odométriques
 * et des données issus d'un scanner laser.\n
 *
 * Le nombre de méthodes de cette classe est volontairement succint. Les calculs internes sont délégués à des objets spécifiques.
 */
class KFLocalizator
{
    public:

        /** \ingroup kfl
         *
         * \class IEKFParams
         *
         * \brief KFLocalizator::IEKFParams rassemble les paramètres relatif au filtre de kalman étendu itératif.
         *
         *  Ce filtre sert de méthode de fusion des données odo et laser.
         */
        class IEKFParams
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  \li defaultOdoVelTransSigma = 0.001
             *  \li defaultOdoVelRotSigma = 0.01
             *  \li defaultLaserRangeSigma = 0.005
             *  \li defaultLaserThetaSigma = 0.05
             *  \li iekfMaxIt = 10
             *  \li iekfInnovationMin = 0.00015
             */
            IEKFParams();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo() const;

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir :\n
             * \li defaultOdoVelTransSigma doit être strictement positif.
             * \li defaultOdoVelRotSigma doit être strictement positif.
             * \li defaultLaserRangeSigma doit être strictement positif.
             * \li defaultLaserThetaSigma doit être strictement positif.
             * \li iekfMaxIt doit être strictement positif.
             * \li iekfInnovationMin doit être strictement positif.
             */
            bool checkConsistency() const;

            /**
             * La précision estimée de la vitesse de translation odo en m/s.\n
             * Il s'agit de l'écart type.
             */
            double defaultOdoVelTransSigma;

            /**
             * La précision estimée de la vitesse de rotation odo en rad/s.\n
             * Il s'agit de l'écart type.
             */
            double defaultOdoVelRotSigma;

            /**
             * La précision estimée de la mesure de distance du laser en m.\n
             * Il s'agit de l'écart type.
             */
            double defaultLaserRangeSigma;

            /**
             * La précision estimée de la mesure d'angle du laser en rad.\n
             * Il s'agit de l'écart type.
             */
            double defaultLaserThetaSigma;

            /**
             * Le nombre d'itérations maximal de la routine itérative du kalman étendu itératif.\n
             * Au delà de ce nombre d'itérations, même si l'estimée n'a pas convergé, l'algo rend la dernière estimée.
             */
            unsigned int iekfMaxIt;

            /**
             * L'innovation minimale désirée pour le IEKF.\n
             * Il s'agit de la norme au carré de l'innovation minimale.
             */
            double iekfInnovationMin;
        };


        /** \ingroup kfl
         *
         * \class Params
         *
         * \brief KFLocalizator::Params rassemble la totalité des paramètres relatif à la Localisation.
         *
         *  Cette classe fait appel à d'autres (sous-)classes de paramètres..
         */
        class Params
        {
            public:
                /** Constructeur par défault.
                 *  Il initialise des paramètres classiques non-stupides :\n
                 *  \li bufferSize = 100
                 *  \li maxTime4OdoPrediction = 0.5
                 *  \li referencedBeacons est vide
                 *  \li initParams => voir KFLocalizator::InitParams()
                 *  \li iekfParams => voir KFLocalizator::IEKFParams()
                 *  \li procParams => voir BeaconDetector::Params()
                 */
                Params();

                /**
                 * Permet de formatter les paramètres en un message lisible.
                 */
                std::string getInfo() const;

                /**
                 * Permet de vérifier que les paramètres sont consistants.\n
                 * A savoir :\n
                 * \li bufferSize est de taille 10 au moins.
                 * \li maxTime4OdoPrediction est strictement positif.
                 * \li chaque sous-groupe de paramètres est consistant.
                 */
                bool checkConsistency() const;

                /**
                 * Taille du buffer interne.\n
                 * Ce buffer stocke des informations temporelles (les différentes estimées). Il est utilisé
                 * afin de pouvoir "revenir dans le passé" pour réaliser une mise à jour a posteriori.\n
                 * La taille de ce buffer doit être suffisemment grande pour que l'algo soit capable de
                 * revenir de Tscan en arrière.\n
                 * bufferSize doit être supérieur à Tscan/Todo.
                 */
                unsigned int bufferSize;

                /**
                 * Si la date de l'estimée Odo est plus récente de plus de maxTime4OdoPrediction (en seconde) par rapport
                 * à la dernière estimée (qui peut être celle de l'initialisation), alors la prediction Odo
                 * est annulée, on estime qu'il y a un pb quelque part.
                 */
                double maxTime4OdoPrediction;

                /**
                 * Position du repère odo dans le ropère de référence du robot
                 */
                arp_math::Pose2D H_odo_robot;

                /**
                 * Position du repère hky dans le ropère de référence du robot
                 */
                arp_math::Pose2D H_hky_robot;

                /**
                 * Covariance de l'initialisation par défault
                 */
                arp_math::Covariance3 defaultInitCovariance;

                /**
                 * Position des balises (circulaires) sur la table
                 */
                std::vector< lsl::Circle > referencedBeacons;

                /**
                 * Paramètres relatifs au filtrage de kalman étendu itératif (niveau de confiance envers le capteurs etc...)
                 */
                kfl::KFLocalizator::IEKFParams iekfParams;

                /**
                 * Paramètres relatifs au traitement de scan (seuils de traitement pour éviter les perturbations etc...)
                 */
                kfl::BeaconDetector::Params procParams;
        };

        struct DebugInfo
        {
                arp_math::Vector2 meas;
                arp_math::Vector2 target;
                long double date;
        };

    public:
        /** Constructeur par défault.
         *  Il initialise avec des paramètres par défault.
         */
        KFLocalizator();

        /**
         * Destructeur par défault
         */
        ~KFLocalizator();

        /**
         * Modification de tous les paramètres
         */
        void setParams(KFLocalizator::Params);

        /**
         * Modification des paramètres relatifs au filtre de kalman.
         */
        void setParams(KFLocalizator::IEKFParams);

        /**
         * Modification des paramètres relatifs au traitement du scan.
         */
        void setParams(BeaconDetector::Params);

        /**
         * Récupération des paramètres courants
         */
        KFLocalizator::Params getParams();

        /**
         * Initialisation de la Localisation.
         * \param pose la pose et sa covariance
         */
        bool initialize(const arp_math::EstimatedPose2D & H_robot_table);

        /**
         * Cette méthode sert à donner au localisateur une nouvelle mesure odo.
         * \param[in] T_odo_table_p_odo_r_odo la mesure odo sous forme d'une estimation de Twist2D. Cette estimation est datée.\n
         * Il s'agit du Twist du repère odo par rapport au repère table, projeté et réduit dans le repère odo.
         */
        bool newOdoVelocity(arp_math::EstimatedTwist2D T_odo_table_p_odo_r_odo);

        /**
         * Cette méthode sert à donner au localisateur une nouvelle mesure laser.
         * \param[in] scan la mesure laser sous forme d'un LaserScan. Le scan est daté.
         */
        int newScan(lsl::LaserScan scan);

        /**
         * Permet d'accéder à la dernière estimée de position.\n
         * Il s'agit de H_robot_table
         * \return EstimatedPose2D correspondant à H_robot_table
         */
        arp_math::EstimatedPose2D getLastEstimatedPose2D();

        /**
         * Permet d'accéder à la dernière estimée de vitesse.\n
         * Il s'agit de T_robot_table_p_robot_r_robot c'est à dire le Twist du robot
         * par rapport à la table, projeté ET réduit dans le repère du robot.
         * \return EstimatedTwist2D correspondant à T_robot_table_p_robot_r_robot
         */
        arp_math::EstimatedTwist2D getLastEstimatedTwist2D();

        /**
         * Permet de récupérer les derniers obstacles détectés sur la table.
         */
        std::vector< arp_math::Vector2 > getDetectedObstacles();

        /**
         * Permet d'obtenir un rapport sur les timings
         */
        std::string getPerformanceReport();

        /**
         * Renvoie le nombre de balises que l'on est sensé voir
         */
        unsigned int getTheoricalVisibility();

        std::vector<DebugInfo> getDebugInfo();


    protected:
        /**
         * Cet objet rassemble la totalité des paramètres nécessaire au localisateur.
         */
        KFLocalizator::Params params;

        /**
         * Cet objet est utilisé pour réaliser le traitement de scan
         */
        BeaconDetector beaconDetector;

        /**
         * Ce pointeur permet d'accéder au framework de filtrage.
         */
        BayesianWrapper * bayesian;

        /**
         * Les derniers obstacles détectés sur le terrain.
         */
        std::vector< arp_math::Vector2 > detectedObstacles;

        /**
         * Buffer circulaire stockant les estimations de Pose et de Twist
         */
        boost::circular_buffer< std::pair< arp_math::EstimatedPose2D, arp_math::EstimatedTwist2D > > circularBuffer;

        /**
         * angle mini du dernier scan
         */
        double min_angle;

        /**
         * angle maxi du dernier scan
         */
        double max_angle;

        arp_core::StatTimer newOdoVelTimer;
        arp_core::StatTimer newScanGlobalTimer;
        arp_core::StatTimer newScanBITPTimer;
        arp_core::StatTimer newScanPreUpdateTimer;
        arp_core::StatTimer newScanUpdateTimer;

        std::vector<DebugInfo> debugInfos;

    protected:
        /**
         * Mets à jour le buffer circulaire
         * \param[in] date de l'estimation car la boite à outil BFL ne connait pas l'heure
         * \param[in] la vitesse si elle est connue
         */
        void updateBuffer(const long double & date, const arp_math::EstimatedTwist2D & t = arp_math::EstimatedTwist2D());

        /**
         * Vide le buffer circulaire de toutes les estimées postérieures à une certaine date
         * \param[in] date La date en question
         */
        void popBufferUntilADate(const double date);

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLOCALIZATOR_HPP_ */
