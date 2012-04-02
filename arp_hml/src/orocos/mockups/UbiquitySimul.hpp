/*
 * UbiquitySimul.hpp
 *
 *  Created on: 02 April 2012
 *      Author: wla
 *
 *  This is the simulation of Ubiquity's hardware interface
 */

#ifndef UBIQUITYSIMUL_HPP_
#define UBIQUITYSIMUL_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <math/core>

using namespace arp_math;

namespace arp_hml
{

    class UbiquitySimul: public HmlTaskContext
    {
    public:
        UbiquitySimul(const std::string& name);

    protected:
        /**
         * Redefined to prevent getOperation from inheritance
         */
        bool configureHook();

        /**
         *
         */
        void updateHook();


        /** Command to be used in position mode. It must be provided in rad on the reductor's output.
           * It is not available yet. */
          InputPort<double> inLeftSteeringPositionCmd;
          InputPort<double> inRightSteeringPositionCmd;
          InputPort<double> inRearSteeringPositionCmd;
          /** Command to be used in speed mode. It must be provided in rad/s on the reductor's output **/
          InputPort<double> inLeftDrivingSpeedCmd;
          InputPort<double> inRightDrivingSpeedCmd;
          InputPort<double> inRearDrivingSpeedCmd;

          /** Provides the real position of the robot taking into account the commands **/
          OutputPort<Pose2D> outRealPosition;

    };

}

#endif
