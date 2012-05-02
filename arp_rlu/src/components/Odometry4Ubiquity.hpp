/*
 * Odometry4Ubiquity.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ODOMETRY4UBIQUITY_HPP_
#define ODOMETRY4UBIQUITY_HPP_

#include "RluTaskContext.hpp"
#include <math/core>
#include <models/core>

namespace arp_rlu
{

class Odometry4Ubiquity: public RluTaskContext
{
    public:
        Odometry4Ubiquity(const std::string& name);

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

        /** The KernelQuality when getting the report fromthe model must be larger than this property, else we spawn an error*/
        double propMinKernelQuality;
        /** Norme minimale du Twist résultant des calculs d'ocométrie, en mm/s */
        double propMinVelocity;

        RTT::InputPort<timespec> inTime;
        RTT::InputPort<arp_model::UbiquityParams> inParams;
        /** Measures from HML */
        RTT::InputPort<arp_model::MotorState> inMotorState;
        /** Computed Twist */
        RTT::OutputPort<arp_math::EstimatedTwist2D> outTwist;
        /** Quality of last computed Twist */
        RTT::OutputPort<double> outKernelQuality;
        /** Slippage detected */
        RTT::OutputPort<bool> outSlippageDetected;

        /**
         * Permet d'ajouter port/operations à l'interface Orocos.
         * Utile afin d'éviter un gros bloc de code non fonctionnel en debut de fichier.
         */
        void createOrocosInterface();
};

} /* namespace arp_rlu */
#endif /* ODOMETRY4UBIQUITY_HPP_ */
