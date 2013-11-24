/*
 * LaserOnlyLocalizatorCpn.hpp
 *
 *  Created on: 9 mai 2012
 *      Author: ard
 */

#ifndef LASERONLYLOCALIZATORCPN_HPP_
#define LASERONLYLOCALIZATORCPN_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

#include <sensor_msgs/LaserScan.h>

#include "KFL/LaserOnlyLocalizator.hpp"


namespace arp_rlu
{

typedef kfl::LaserOnlyLocalizator::Params LaserOnlyLocalizatorParams;


class LaserOnlyLocalizatorCpn: public RluTaskContext
{
    public:
        LaserOnlyLocalizatorCpn(const std::string& name);
        void setParams(LaserOnlyLocalizatorParams params);
        std::string printParams();
        bool configureHook();
        void updateHook();

    protected:
        //*****************************************************
        // Params
        LaserOnlyLocalizatorParams propParams;

        //*****************************************************
        // Ports
        RTT::InputPort<sensor_msgs::LaserScan> inScan;



        //*****************************************************
        // Operations
        virtual void ooGetPerformanceReport();

        void ooSwitchToRedConfig();
        void ooSwitchToYellowConfig();

        /**
         *
         */
        bool ooDo();

        /**
         *  Permet d'accéder à l'estimée calculée si le calcul a réussi.\n
         *  Il s'agit de H_hky_table
         */
        arp_math::EstimatedPose2D ooGetEstimatedPose();

        /**
         *  Conseille un cap relatif à effecter afin de confirmer/améliorer le résultat.
         *  \li Si le calcul a réussi, le cap permet de placer le robot afin qu'il puisse voir 3 balises.
         *  \li Si le calcul a échoué, le cap est de +60°.
         */
        double ooGetRelativeHeadingForConfirmation();


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
        kfl::LaserOnlyLocalizator loloc;
        long double m_monotonicTimeToRealTime;
        lsl::LaserScan lslScan;
        bool lastSuccess;


};

} /* namespace arp_rlu */
#endif /* LASERONLYLOCALIZATORCPN_HPP_ */
