/*
 * UbiquityStates.hpp
 *
 *  Created on: April 6 2012
 *      Author: ard
 *
 *      Ce fichier contient toutes les structures utilisées par les modèles pour ragroupes les informations d'état.
 */

#ifndef UBIQUITYSTATES_HPP_
#define UBIQUITYSTATES_HPP_

#include <iostream>
#include <sstream>

namespace arp_model
{

class AxeState
{
    public:
        double position;
        double velocity;
        double torque;

        AxeState():
            position(0.0),
            velocity(0.0),
            torque(0.0)
        {}

        std::string toString(int slash = 0) const
        {
            std::stringstream ss;
            ss  << std::endl;
            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "position : " << position << std::endl;

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "velocity : " << velocity << std::endl;

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "torque : " << torque << std::endl;

            ss << std::endl;

            return ss.str();
        }
};

class AxesGroup
{
    public:
        AxeState left;
        AxeState right;
        AxeState rear;

        AxesGroup():
            left(),
            right(),
            rear()
        {}

        std::string toString(int slash = 0) const
        {
            std::stringstream ss;
            ss  << std::endl;
            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "left : " << left.toString(slash+1) << std::endl;

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "right : " << right.toString(slash+1) << std::endl;

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "rear " << rear.toString(slash+1) << std::endl;

            ss << std::endl;

            return ss.str();
        }
};

/** Don't use this struct, prefer the two TurretState and Motor state below */
class UbiquityKinematicState
{
    public:
        AxesGroup steering;
        AxesGroup driving;

        UbiquityKinematicState():
            steering(),
            driving()
        {}

        std::string toString() const
        {
            std::stringstream ss;
            ss  << std::endl;
            ss << "Driving : " << driving.toString(1)
                << "Steering : " << steering.toString(1);
            ss << std::endl;
            return ss.str();
        }
};

class TurretState: public UbiquityKinematicState
{
};

class MotorState: public UbiquityKinematicState
{
};



class SlippageReport
{
    public:
        double kernelQuality;
};
} /* namespace arp_model */
#endif /* UBIQUITYSTATES_HPP_ */
