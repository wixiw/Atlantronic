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
        bool configureHook();
        void updateHook();

    protected:
        //*****************************************************
        // Params
        LocalizatorParams propParams;

        //*****************************************************
        // Ports
        RTT::InputPort<double> inScan;
        RTT::InputPort<arp_math::EstimatedTwist2D > inOdo;

        RTT::OutputPort<arp_math::EstimatedPose2D> outPose;
        RTT::OutputPort<arp_math::EstimatedTwist2D> outTwist;


        //*****************************************************
        // Methods

        /* Cree l'interface Orocos : ajout de port, proprietes, operations */
        void createOrocosInterface();


        //*****************************************************
        // Callbacks

        /** callback appelee lors de la reception de donnees sur inScan,
         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        void scanCb(RTT::base::PortInterface* portInterface);

        /** callback appelee lors de la reception de donnees sur inOdo,
         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        void odoCb(RTT::base::PortInterface* portInterface);

        //*****************************************************
        // Internal objects
        kfl::KFLocalizator kfloc;

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
