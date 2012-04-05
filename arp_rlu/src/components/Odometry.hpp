/*
 * Odometry.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "RluTaskContext.hpp"
#include <math/core>
#include <models/UbiquityParams.hpp>

using namespace arp_math;

namespace arp_rlu
{

class Odometry: public RluTaskContext
{
    public:
        Odometry(const std::string& name);

    protected:
        InputPort<double> inTime;
        InputPort<UbiquityParams> inParams;

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

        OutputPort<EstimatedTwist2D> outTwist;

        /**
         * Permet d'ajouter port/operations à l'interface Orocos.
         * Utile afin d'éviter un gros bloc de code non fonctionnel en debut de fichier.
         */
        void createOrocosInterface();
};

} /* namespace arp_rlu */
#endif /* ODOMETRY_HPP_ */
