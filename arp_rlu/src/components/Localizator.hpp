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
         * Indique si le filtre n'a pas divergé.\n
         * Les termes diaguonaux de la matrice de convergence de l'estimée de position sont
         * comparés aux seuils maxReliableTransStddev and maxReliableRotStddev.\n
         * Si le filtre a divergé, la valeur renvoyée vaut false et true sinon.
         */
        RTT::OutputPort<bool> outReliability;

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


        //*****************************************************
        // Callbacks

        //        /** callback appelee lors de la reception de donnees sur inScan,
        //         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        //        void scanCb(RTT::base::PortInterface* portInterface);
        //
        //        /** callback appelee lors de la reception de donnees sur inOdo,
        //         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        //        void odoCb(RTT::base::PortInterface* portInterface);

        //*****************************************************
        // Internal objects
        kfl::KFLocalizator kfloc;
        long double m_monotonicTimeToRealTime;

        double maxReliableTransStddev;
        double maxReliableRotStddev;

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
