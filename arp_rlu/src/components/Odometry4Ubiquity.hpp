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
#include <models/UbiquityParams.hpp>

namespace arp_rlu
{

class Odometry4Ubiquity: public RluTaskContext
{
    public:
        Odometry4Ubiquity(const std::string& name);

        /** Callback d'update.*/
        virtual void updateHook();

    protected:
        InputPort<double> inTime;
        InputPort<arp_model::UbiquityParams> inParams;

        InputPort<double> inLeftDrivingSpeed;
        InputPort<double> inRightDrivingSpeed;
        InputPort<double> inRearDrivingSpeed;
        InputPort<double> inLeftSteeringSpeed;
        InputPort<double> inRightSteeringSpeed;
        InputPort<double> inRearSteeringSpeed;

        InputPort<double> inLeftDrivingPosition;
        InputPort<double> inRightDrivingPosition;
        InputPort<double> inRearDrivingPosition;
        InputPort<double> inLeftSteeringPosition;
        InputPort<double> inRightSteeringPosition;
        InputPort<double> inRearSteeringPosition;

        OutputPort<arp_math::EstimatedTwist2D> outTwist;

        /**
         * Permet d'ajouter port/operations à l'interface Orocos.
         * Utile afin d'éviter un gros bloc de code non fonctionnel en debut de fichier.
         */
        void createOrocosInterface();
};

} /* namespace arp_rlu */
#endif /* ODOMETRY4UBIQUITY_HPP_ */
