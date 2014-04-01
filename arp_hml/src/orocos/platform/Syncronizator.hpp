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
#include "time/ArdTime.hpp"

namespace arp_hml
{

class Syncronizator: public HmlTaskContext
{
    public:
        Syncronizator(const std::string name);

        void updateHook();

    protected:
        /** Ce port est publié à la fin de la syncronisation, des composants qui sont connectés aux ports de données peuvent
         * se trigger en eventPort sur outClock. Il contient la date des mesures sur le CAN (date de l'envoit du message SYNC)
         */
        OutputPort<arp_time::ArdAbsoluteTime> outClock;

        /**
         * Ce port contient un condensé de toutes les mesures Hml a une date identique pour tous les moteurs.
         */
        OutputPort<arp_model::MotorState> outMotorMeasures;

        InputPort<arp_time::ArdAbsoluteTime> inCanSync;

        InputPort<arp_time::ArdAbsoluteTime> inLeftDrivingClock;
        InputPort<arp_time::ArdAbsoluteTime> inRightDrivingClock;
        InputPort<arp_time::ArdAbsoluteTime> inRearDrivingClock;
        InputPort<arp_time::ArdAbsoluteTime> inLeftSteeringClock;
        InputPort<arp_time::ArdAbsoluteTime> inRightSteeringClock;
        InputPort<arp_time::ArdAbsoluteTime> inRearSteeringClock;

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
        arp_time::ArdAbsoluteTime m_syncTime;
};

} /* namespace arp_hml */
#endif /* SYNCRONIZATOR_HPP_ */
