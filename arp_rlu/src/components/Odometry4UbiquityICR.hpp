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
#include <helpers/SimpsonIntegrator.hpp>

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
        /** Buffer local pour la outPose */
        double attrHeading;
        /** Buffer local pour la outPose */
        arp_math::EstimatedPose2D attrPose;
        /** Buffer local pour les params */
        arp_model::UbiquityParams attrParams;
        /** Buffer local pour le temps */
        timespec attrLastTime;

        RTT::InputPort<timespec> inTime;
        RTT::InputPort<arp_model::UbiquityParams> inParams;
        /** Measures from HML */
        RTT::InputPort<arp_model::MotorState> inMotorState;
        /** Heading from external source */
        RTT::InputPort<double> inTrueHeading;
        /** Pose from external source */
        RTT::InputPort<arp_math::Pose2D> inTruePose;
        /** Computed Twist */
        RTT::OutputPort<arp_math::EstimatedICRSpeed> outICRSpeed;
        RTT::OutputPort<arp_math::Twist2D> outTwist;
        /** Slippage detected */
        RTT::OutputPort<arp_model::SlippageReport> outSlippageDetected;
        /** Estimated Pose */
        RTT::OutputPort<arp_math::EstimatedPose2D> outPose;

        /** Angular speeds from 2 turret speeds */
        RTT::OutputPort<double> outLRiOmega;
        RTT::OutputPort<double> outRiReOmega;
        RTT::OutputPort<double> outReLOmega;

        RTT::OutputPort<double> outICRSphericalPerimeter;

        /**
         * Permet d'ajouter port/operations à l'interface Orocos.
         * Utile afin d'éviter un gros bloc de code non fonctionnel en debut de fichier.
         */
        void createOrocosInterface();

    protected: // internal
        arp_math::SimpsonIntegrator vxIntegrator;
        arp_math::SimpsonIntegrator vyIntegrator;
        arp_math::SimpsonIntegrator vhIntegrator;
};

} /* namespace arp_rlu */
#endif /* ODOMETRY4UBIQUITY_HPP_ */
