/*
 * SimpleLocalizator.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef SIMPLELOCALIZATOR_HPP_
#define SIMPLELOCALIZATOR_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

#include <sensor_msgs/LaserScan.h>
#include "LocalizatorTypes.hpp"
#include <helpers/SimpsonIntegrator.hpp>
#include "time/ArdTime.hpp"

namespace arp_rlu
{

class SimpleLocalizator: public RluTaskContext
{

    public:
        SimpleLocalizator(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        
        //*****************************************************
        // Attributes

        /** Buffer local pour le temps */
        arp_time::ArdAbsoluteTime attrLastTime;

        //*****************************************************
        // Properties

        double propMaxReliableBadOdoTransStddev;
        double propMaxReliableBadOdoRotStddev;

        //*****************************************************
        // Ports

        RTT::InputPort<arp_time::ArdAbsoluteTime> inTime;
        RTT::InputPort<arp_math::EstimatedTwist2D > inTwistOdo;
        RTT::InputPort<double> inGyroAngle;

        /** Heading from external source (for instance initialisation)*/
        RTT::InputPort<double> inTrueHeading;
        /** Pose from external source (for instance initialisation)*/
        RTT::InputPort<arp_math::Pose2D> inTruePose;

        /**
         * Contient la dernière estimée de position.\n
         * Il s'agit de H_robot_table
         */
        RTT::OutputPort<arp_math::EstimatedPose2D> outPose;

        /**
         * Indique l'état de la localization.\n
         */
        RTT::OutputPort<int> outLocalizationState;

        /**
         * Indique le mode de la localization.\n
         */
        RTT::OutputPort<int> outLocalizationMode;

        /**
         * Indique la qualité de l'estimation.\n
         */
        RTT::OutputPort<int> outLocalizationQuality;

        //*****************************************************
        // Operations

        void ooSwitchToRedConfig();
        void ooSwitchToYellowConfig();

        //*****************************************************
        // Methods

        /* Cree l'interface Orocos : ajout de port, proprietes, operations */
        void createOrocosInterface();

        void updateLocalizationStates();

        bool halt();
        bool resume();

        std::string getInfo();


        //*****************************************************
        // Internal objects
        arp_time::ArdTimeDelta m_monotonicTimeToRealTime;

        /** Localizator anyway computing but will only publish in RUNNING state.
         The only way to do it is to initialize it with a true position */
        LocalizationState currentState;
        LocalizationMode currentMode;
        LocalizationQuality currentQuality;

        arp_math::SimpsonIntegrator vxIntegrator;
        arp_math::SimpsonIntegrator vyIntegrator;
        arp_math::SimpsonIntegrator vhIntegrator;

        const arp_math::Covariance3 m_defaultInitCovariance;
};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
