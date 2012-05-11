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

#include "KFL/KFLocalizator.hpp"

namespace arp_rlu
{

typedef kfl::KFLocalizator::Params LocalizatorParams;

enum LocalizationState
{
    __STOPED__ = 0,
    _ODO_ONLY_ = 1,
    __FUSION__ = 2,
    _BAD__ODO_ = 3,
    ___LOST___ = 4
};

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
        RTT::InputPort<arp_math::EstimatedTwist2D > inOdo;
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
        RTT::OutputPort<arp_math::EstimatedTwist2D> outTwist;

        /**
         * Indique l'état de la localization.\n
         */
        RTT::OutputPort<LocalizationState> outLocalizationState;

        /**
         * Contient la position d'obstacles détectés sur la table.\n
         * Ces obstacles ne sont pas filtrés temporellement.
         */
        RTT::OutputPort< std::vector< arp_math::Vector2 > > outObstacles;

        //*****************************************************
        // Operations
        virtual std::string coGetPerformanceReport();

        void ooSwitchToRedConfig();
        void ooSwitchToPurpleConfig();


        //*****************************************************
        // Methods

        /* Cree l'interface Orocos : ajout de port, proprietes, operations */
        void createOrocosInterface();

        void updateLocalizationState();


        //*****************************************************
        // Internal objects
        kfl::KFLocalizator kfloc;
        long double m_monotonicTimeToRealTime;

        double propMaxReliableTransStddev;
        double propMaxReliableRotStddev;

        double propLaserRangeSigma;
        double propLaserThetaSigma;

        double propLaserRangeSigmaSmooth;
        double propLaserThetaSigmaSmooth;

        unsigned int propLostCptThreshold;

        bool updateTried;
        bool predictionOk;
        bool updateOk;

        LocalizationState currentState;
        unsigned int lostCpt;

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
