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
        /** Ce port est publié à la fin de la syncronisation, des composants qui sont connectés aux ports de données peuvent
         * se trigger en eventPort sur outClock. Il contient la date des mesures sur le CAN (date de l'envoit du message SYNC)
         */
        OutputPort<timespec> outClock;

        /**
         * Ce port contient un condensé de toutes les mesures Hml a une date identique pour tous les moteurs.
         */
        OutputPort<arp_model::MotorState> outMotorMeasures;

        InputPort<timespec> inCanSync;

        InputPort<timespec> inLeftDrivingClock;
        InputPort<timespec> inRightDrivingClock;
        InputPort<timespec> inRearDrivingClock;
        InputPort<timespec> inLeftSteeringClock;
        InputPort<timespec> inRightSteeringClock;
        InputPort<timespec> inRearSteeringClock;

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

        /**
         * Lis les ports d'entrée Orocos et construit la MotorState
         */
        arp_model::MotorState readInputs();

        /** construit l'interface Orocos */
        void createOrocosInterface();

        /** contient la date de l'horloge principale */
        timespec m_syncTime;
};

} /* namespace arp_hml */
#endif /* SYNCRONIZATOR_HPP_ */
