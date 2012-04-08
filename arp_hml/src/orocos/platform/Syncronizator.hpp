/*
 * Syncronizator.hpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 *
 *  Ce composant sert à resyncroniser les sorties des 6 moteurs après la syncro.
 */

#ifndef SYNCRONIZATOR_HPP_
#define SYNCRONIZATOR_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <models/UbiquityKinematics.hpp>
#include <boost/thread.hpp>

namespace arp_hml
{

class Syncronizator: public arp_hml::HmlTaskContext
{
    public:
        Syncronizator(const std::string name);

        void updateHook();

    protected:
        /**
         * Compte le nombre de message de syncronisation venant des moteurs qui ne sont pas dans le cycle courant
         */
        int attrNbError;

        /**
         * Set this to true to have detailled log of synchronisation
         */
        bool propVerbose;

        /** Ce port est publié à la fin de la syncronisation, des composants qui sont connectés aux ports de données peuvent
         * se trigger en eventPort sur outClock. Il contient la date des mesures sur le CAN (date de l'envoit du message SYNC)
         */
        OutputPort<double> outClock;

        /**
         * Ce port contient un condensé de toutes les mesures Hml a une date identique pour tous les moteurs.
         */
        OutputPort<arp_model::MotorState> outMotorMeasures;

        InputPort<double> inCanSync;

        InputPort<double> inLeftDrivingClock;
        InputPort<double> inRightDrivingClock;
        InputPort<double> inRearDrivingClock;
        InputPort<double> inLeftSteeringClock;
        InputPort<double> inRightSteeringClock;
        InputPort<double> inRearSteeringClock;

        InputPort<double> inLeftDrivingVelocity;
        InputPort<double> inRightDrivingVelocity;
        InputPort<double> inRearDrivingVelocity;
        InputPort<double> inLeftSteeringVelocity;
        InputPort<double> inRightSteeringVelocity;
        InputPort<double> inRearSteeringVelocity;

        InputPort<double> inLeftDrivingPosition;
        InputPort<double> inRightDrivingPosition;
        InputPort<double> inRearDrivingPosition;
        InputPort<double> inLeftSteeringPosition;
        InputPort<double> inRightSteeringPosition;
        InputPort<double> inRearSteeringPosition;

        InputPort<double> inLeftDrivingTorque;
        InputPort<double> inRightDrivingTorque;
        InputPort<double> inRearDrivingTorque;
        InputPort<double> inLeftSteeringTorque;
        InputPort<double> inRightSteeringTorque;
        InputPort<double> inRearSteeringTorque;

        /** callbacks pour les eventPort des horloges des composants à syncroniser
         * BIG FAT WARNING : attention L'updateHook est appelé derrière.
         * ceci ne sera pas valable dans les versions suivantes d'Orocos.
         */
        void eventPortCB(RTT::base::PortInterface* portInterface);

        /** callbacks pour l'eventPort de l'horloge principale
         * BIG FAT WARNING : attention L'updateHook est appelé derrière.
         * ceci ne sera pas valable dans les versions suivantes d'Orocos.
         */
        void eventCanSyncCB(RTT::base::PortInterface* portInterface);

        /**
         * Teste la liste des messages sync reçu pour verifier si l'updateHook doit être executé
         * m_syncCount doit valoir 6 et les syncTime doivent être identiques
         * @param syncTime : si la fonction returne true, alors syncTime contient le temps de syncro, sinon il vaut -1
         */
        bool isAllSyncReveived(double& syncTime);

        /**
         * Lis les ports d'entrée Orocos et construit la MotorState
         */
        arp_model::MotorState readInputs();

        /** construit l'interface Orocos */
        void createOrocosInterface();

        /** contient la date de l'horloge principale */
        double m_syncTime;

        /** compte le nombre de message sync recu depuis le dernier updateHook
         * Chaque bit est mis à 1 ou 0 en fonction du moteur dont on a reçu la sync
         * on a eut toutes les sync si m_syncCount = 0b111111
         */
        int m_syncCount;
};

} /* namespace arp_hml */
#endif /* SYNCRONIZATOR_HPP_ */
