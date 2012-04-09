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

            return ss.str();
        }

        AxeState operator+(const AxeState& b) const
        {
            AxeState res;
            res.position = position + b.position;
            res.velocity = velocity + b.velocity;
            res.torque = torque + b.torque;
            return res;
        }

        AxeState operator-(const AxeState& b) const
        {
            AxeState res;
            res.position = position - b.position;
            res.velocity = velocity - b.velocity;
            res.torque = torque - b.torque;
            return res;
        }

        AxeState operator*(const double& scal) const
        {
            AxeState res;
            res.position = position * scal;
            res.velocity = velocity * scal;
            res.torque = torque * scal;
            return res;
        }

        AxeState operator/(const double& scal) const
        {
            AxeState res;
            res.position = position / scal;
            res.velocity = velocity / scal;
            res.torque = torque / scal;
            return res;
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
            ss << "left : " << left.toString(slash+1);

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "right : " << right.toString(slash+1);

            for( int i = 0 ; i < slash ; i++ )
                ss  << "\t";
            ss << "rear " << rear.toString(slash+1);

            return ss.str();
        }

        AxesGroup operator+(const AxesGroup& b) const
        {
            AxesGroup group;
            group.left = left + b.left;
            group.right = right + b.right;
            group.rear = rear + b.rear;
            return group;
        }

        AxesGroup operator-(const AxesGroup& b) const
        {
            AxesGroup group;
            group.left = left - b.left;
            group.right = right - b.right;
            group.rear = rear - b.rear;
            return group;
        }

        AxesGroup operator*(const double& scal) const
        {
            AxesGroup group;
            group.left = left * scal;
            group.right = right * scal;
            group.rear = rear * scal;
            return group;
        }

        AxesGroup operator/(const double& scal) const
        {
            AxesGroup group;
            group.left = left / scal;
            group.right = right / scal;
            group.rear = rear / scal;
            return group;
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

        UbiquityKinematicState operator+(const UbiquityKinematicState& b) const
        {
            UbiquityKinematicState state;
            state.steering = steering + b.steering;
            state.driving = driving + b.driving;
            return state;
        }

        UbiquityKinematicState operator-(const UbiquityKinematicState& b) const
        {
            UbiquityKinematicState state;
            state.steering = steering - b.steering;
            state.driving = driving - b.driving;
            return state;
        }

        UbiquityKinematicState operator*(const double& scal) const
        {
            UbiquityKinematicState state;
            state.steering = steering * scal;
            state.driving = driving * scal;
            return state;
        }

        UbiquityKinematicState operator/(const double& scal) const
        {
            UbiquityKinematicState state;
            state.steering = steering / scal;
            state.driving = driving / scal;
            return state;
        }
};

typedef UbiquityKinematicState TurretState;
typedef UbiquityKinematicState MotorState;


class SlippageReport
{
    public:
        double kernelQuality;
};
} /* namespace arp_model */
#endif /* UBIQUITYSTATES_HPP_ */
