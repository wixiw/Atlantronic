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

namespace arp_rlu
{

namespace kfl
{
/** \ingroup kfl
 * \nonstableyet
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
         * \nonstableyet
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
         * \nonstableyet
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
         * \nonstableyet
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
                 *  bufferSize = 100\n
                 *  initParams => voir KFLocalizator::InitParams() \n
                 *  iekfParams => voir KFLocalizator::InitParams() \n
                 *  procParams => voir BeaconDetector::Params() \n
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
        bool setParams(KFLocalizator::InitParams);

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
         * \param time date en seconde de la mesure odo
         * \param odoVel la mesure odo sous forme d'un Twist2D
         * \remarks Si le Twist2D s'avère être un EstimatedTwist2D (test via un dynamic_cast),
         * les valeurs de covariance de l'EstimatedTwist2D sont utilisées en lieu et place des valeurs
         * spécifiée par iekfParams.
         * \remarks En revanche, si le Twist2D s'avère être un EstimatedTwist2D, la date
         * utilisée par le localisateur est bien celle spécifiée en premier argument et non celle spécifiée
         * dans le EstimatedTwist2D.
         */
        bool newOdoVelocity(double time, arp_math::Twist2D odoVel);

        /**
         * Cette méthode sert à donner au localisateur une nouvelle mesure laser.
         * \param time date en seconde correspondant à la dernière mesure du scan laser
         * \param scan la mesure laser sous forme d'un LaserScan
         * \remarks La date utilisée pour la mise à jour provient non pas du premier argument
         * mais du champ de date du LaserScan. La date spécifiée dans le premier argument est
         * seulement utilisé pour identifier la partie du buffer à utiliser.
         */
        bool newScan(double time, lsl::LaserScan scan);

        /**
         * Permet d'accéder à la dernière (et donc la meilleure) estimée de position.
         * \return EstimatedPose2D
         */
        arp_math::EstimatedPose2D getPose2D();

        /**
         * Permet d'accéder à la dernière (et donc la meilleure) estimée de vitesse.
         * \return EstimatedTwist2D
         */
        arp_math::EstimatedTwist2D getTwist2D();


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



};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLOCALIZATOR_HPP_ */
