/*
 * Odometry4UbiquityICR.hpp
 *
 *  Created on: Sept 29, 2013
 *      Author: ard
 */

#ifndef ODOMETRY4UBIQUITYICR_HPP_
#define ODOMETRY4UBIQUITYICR_HPP_

#include "RluTaskContext.hpp"
#include <math/core>
#include <models/core>

namespace arp_rlu
{

class Odometry4UbiquityICR: public RluTaskContext
{
    public:
        Odometry4UbiquityICR(const std::string& name);

        /** Callback d'update.*/
        virtual void updateHook();

    protected:
        /**Internal model feedback, for debug info only */
        arp_model::TurretState attrTurretState;
        /** Buffer local pour les inMotorState */
        arp_model::MotorState attrMotorState;
        /** Buffer local pour les params */
        arp_model::UbiquityParams attrParams;
        /** Buffer local pour le temps */
        timespec attrTime;

        RTT::InputPort<timespec> inTime;
        RTT::InputPort<arp_model::UbiquityParams> inParams;
        /** Measures from HML */
        RTT::InputPort<arp_model::MotorState> inMotorState;
        /** Computed Twist */
        RTT::OutputPort<arp_math::EstimatedICRSpeed> outICRSpeed;
        /** Slippage detected */
        RTT::OutputPort<arp_model::SlippageReport> outSlippageDetected;

        /**
         * Permet d'ajouter port/operations à l'interface Orocos.
         * Utile afin d'éviter un gros bloc de code non fonctionnel en debut de fichier.
         */
        void createOrocosInterface();
};

} /* namespace arp_rlu */
#endif /* ODOMETRY4UBIQUITY_HPP_ */
