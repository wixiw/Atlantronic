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

using namespace arp_math;

namespace arp_rlu
{

struct LocalizatorParams
{
        double truc;

        //valeurs par defaut pas trop debiles
        //ou completement debiles suivant les philosophies
        LocalizatorParams():
            truc(1.0)
        {}

        bool check() const
        {
            if( truc <= 0 )
                return false;

            //tout est ok
            return true;
        }
};


class Localizator: public RluTaskContext
{
    public:
        Localizator(const std::string& name);
        bool initialize(EstimatedPose2D pose);
        void setParams(LocalizatorParams params);
        void updadeHook();

    protected:
        LocalizatorParams propParams;

        InputPort<double> inScan;
        InputPort<EstimatedTwist2D > inOdo;

        OutputPort<EstimatedPose2D> outPose;
        OutputPort<EstimatedTwist2D> outTwist;

        /* Cree l'interface Orocos : ajout de port, proprietes, operations */
        void createOrocosInterface();

        /** callback appelee lors de la reception de donnees sur inScan,
         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        void scanCb(RTT::base::PortInterface* portInterface);

        /** callback appelee lors de la reception de donnees sur inOdo,
         * Attention l'updateHook sera automatique appelee... jusqu'au prochaine versions d'orocos*/
        void odoCb(RTT::base::PortInterface* portInterface);

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
