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

    protected:
        LocalizatorParams propParams;

        InputPort<double> inScan;
        InputPort<EstimatedTwist2D > inOdo;

        OutputPort<EstimatedPose2D> outPose;
        OutputPort<EstimatedTwist2D> outTwist;

        void createOrocosInterface();

};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
