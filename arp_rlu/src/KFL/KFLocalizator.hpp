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
         * \class InitParams
         *
         * \brief KFLocalizator::InitParams rassemble les paramètres d'initialisation du localisateur.
         *
         */
        class InitParams
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  (x,y,h) = (0., 0., 0.)\n
             *  t = now()\n
             *  cov = diag( [0.1, 0.1, 0.01] )
             */
            InitParams();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * La pose initiale.\n
             * Il s'agit d'une pose estimée : outre la position x,y,h elle comprend un temps et
             * une matrice de covariance (représentant la confiance).
             */
            arp_math::EstimatedPose2D initialPose;
        };

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
                 *  defaultOdoVelTransSigma = \n
                 *  defaultOdoVelRotSigma = \n
                 *  defaultLaserRangeSigma = \n
                 *  defaultLaserThetaSigma = \n
                 *  iekfMaxIt = \n
                 */
                IEKFParams();

                /**
                 * Permet de formatter les paramètres en un message lisible.
                 */
                std::string getInfo();

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
                 *  \li bufferSize = 100\n
                 *  \li referencedBeacons contient 3 balises
                 *  \li initParams => voir KFLocalizator::InitParams() \n
                 *  \li iekfParams => voir KFLocalizator::IEKFParams() \n
                 *  \li procParams => voir BeaconDetector::Params() \n
                 */
                Params();

                /**
                 * Permet de formatter les paramètres en un message lisible.
                 */
                std::string getInfo();

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
                 * Position des balises (circulaires) sur la table
                 */
                std::vector< lsl::Circle > referencedBeacons;

                /**
                 * Paramètres relatifs à l'initialisation de la localisation (position initiale etc...)
                 */
                kfl::KFLocalizator::InitParams initParams;

                /**
                 * Paramètres relatifs au filtrage de kalman étendu itératif (niveau de confiance envers le capteurs etc...)
                 */
                kfl::KFLocalizator::IEKFParams iekfParams;

                /**
                 * Paramètres relatifs au traitement de scan (seuils de traitement pour éviter les perturbations etc...)
                 */
                kfl::BeaconDetector::Params procParams;
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
         * Modification des paramètres d'initialisation
         */
        void setParams(KFLocalizator::InitParams);

        /**
         * Modification des paramètres relatifs au filtre de kalman.
         */
        void setParams(KFLocalizator::IEKFParams);

        /**
         * Modification des paramètres relatifs au traitement du scan.
         */
        void setParams(BeaconDetector::Params);

        /**
         * Initialisation de la Localisation.\n
         * \remarks Les paramètres d'init (InitParams) sont utilisés pour initialiser.
         */
        bool initialize();

        /**
         * Cette méthode sert à donner au localisateur une nouvelle mesure odo.
         * \param[in] odoVel la mesure odo sous forme d'une estimation de Twist2D. Cette estimation est datée.
         */
        bool newOdoVelocity(arp_math::EstimatedTwist2D odoVel);

        /**
         * Cette méthode sert à donner au localisateur une nouvelle mesure laser.
         * \param[in] scan la mesure laser sous forme d'un LaserScan. Le scan est daté.
         */
        bool newScan(lsl::LaserScan scan);

        /**
         * Permet d'accéder à la dernière estimée de position.
         * \return EstimatedPose2D
         */
        arp_math::EstimatedPose2D getLastEstimatedPose2D();

        /**
         * Permet d'accéder à la dernière estimée de vitesse.
         * \return EstimatedTwist2D
         */
        arp_math::EstimatedTwist2D getLastEstimatedTwist2D();


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
         * Buffer circulaire stockant les estimations de Pose et de Twist
         */
        boost::circular_buffer< std::pair< arp_math::EstimatedPose2D, arp_math::EstimatedTwist2D > > circularBuffer;

    protected:
        /**
         * Mets à jour le buffer circulaire
         * \param[in] date de l'estimation car la boite à outil BFL n'a pas à connaitre l'heure
         * \param[in] la vitesse si elle est connue
         */
        void updateBuffer(const double date, const arp_math::EstimatedTwist2D & t = arp_math::EstimatedTwist2D());


};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLOCALIZATOR_HPP_ */
