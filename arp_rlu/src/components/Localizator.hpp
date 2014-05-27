/*
 * Localizator.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef LOCALIZATOR_HPP_
#define LOCALIZATOR_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

#include <sensor_msgs/LaserScan.h>
#include "LocalizatorTypes.hpp"
#include "KFL/KFLocalizator.hpp"
#include "time/ArdTime.hpp"

namespace arp_rlu
{

typedef kfl::KFLocalizator::Params LocalizatorParams;

class Localizator: public RluTaskContext
{

    public:
        Localizator(const std::string& name);
        bool ooInitialize(double x, double y, double theta);
        void setParams(LocalizatorParams params);
        std::string printParams();
        bool configureHook();
        void updateHook();

    protected:
        //*****************************************************
        // Params
        LocalizatorParams propParams;

        //*****************************************************
        // Ports
        RTT::InputPort<sensor_msgs::LaserScan> inScan;
        RTT::InputPort<arp_math::EstimatedICRSpeed > inOdo;
        RTT::InputPort<double> inGyroAngle;
        RTT::InputPort< bool > inSmoothMode;


        /**
         * Contient la dernière estimée de position.\n
         * Il s'agit de H_robot_table
         */
        RTT::OutputPort<arp_math::EstimatedPose2D> outPose;

        /**
         * Contient la dernière estimée de vitesse.\n
         * Il s'agit de T_robot_table_p_robot_r_robot c'est à dire le Twist du robot
         * par rapport à la table, projeté ET réduit dans le repère du robot.
         */
        RTT::OutputPort<arp_math::EstimatedICRSpeed> outICRSpeed;

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

        /**
         * Indique la qualité de la visibility.\n
         */
        RTT::OutputPort<int> outLocalizationVisibility;

        /**
         * Contient la position d'obstacles détectés sur la table.\n
         * Ces obstacles ne sont pas filtrés temporellement.
         */
        RTT::OutputPort< std::vector< arp_math::Vector2 > > outObstacles;


        RTT::OutputPort< int > outNbSeenBeacons;


        //*****************************************************
        // Operations
        virtual void ooGetPerformanceReport();

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
        kfl::KFLocalizator kfloc;
        arp_time::ArdTimeDelta m_monotonicTimeToRealTime;

        double propMaxReliableBadOdoTransStddev;
        double propMaxReliableBadOdoRotStddev;
        double propMaxReliableLostTransStddev;
        double propMaxReliableLostRotStddev;

        double propLaserRangeSigma;
        double propLaserThetaSigma;

        double propLaserRangeSigmaSmooth;
        double propLaserThetaSigmaSmooth;

        unsigned int propIEKFMaxIt;
        double propIEKFInnovationMin;

        double propCornerBeaconX;
        double propCornerBeaconY;
        double propMiddleBeaconX;
        double propMiddleBeaconY;
        double propBeaconRadius;

        /** Activate or not the time reporting */
        bool propTimeReporting;

        bool smoothMode;
        bool predictionOk;
        bool updateOk;
        int nbSeenBeacons;

        LocalizationState currentState;
        LocalizationMode currentMode;
        LocalizationQuality currentQuality;
        LocalizationVisibility currentVisibility;

        arp_core::StatTimer m_timer;

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
