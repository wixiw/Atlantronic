/*
 * HmlCmdMockup.hpp
 *
 *  Created on: 03 Fev 2012
 *      Author: wla
 *
 *  This is a mockup de replace Hml instructors that should be connected to Hml to give orders
 */

#ifndef HMLCMDMOCKUP_HPP_
#define HMLCMDMOCKUP_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"

namespace arp_hml
{

    class HmlCmdMockup: public HmlTaskContext
    {
    public:
        HmlCmdMockup(const std::string& name);

        OutputPort<double> outLeftDrivingSpeedCmd;
        OutputPort<double> outRightDrivingSpeedCmd;
        OutputPort<double> outRearDrivingSpeedCmd;
        OutputPort<double> outLeftSteeringSpeedCmd;
        OutputPort<double> outRightSteeringSpeedCmd;
        OutputPort<double> outRearSteeringSpeedCmd;

        OutputPort<double> outLeftDrivingPositionCmd;
        OutputPort<double> outRightDrivingPositionCmd;
        OutputPort<double> outRearDrivingPositionCmd;
        OutputPort<double> outLeftSteeringPositionCmd;
        OutputPort<double> outRightSteeringPositionCmd;
        OutputPort<double> outRearSteeringPositionCmd;

        OutputPort<double> outLeftDrivingTorqueCmd;
        OutputPort<double> outRightDrivingTorqueCmd;
        OutputPort<double> outRearDrivingTorqueCmd;
        OutputPort<double> outLeftSteeringTorqueCmd;
        OutputPort<double> outRightSteeringTorqueCmd;
        OutputPort<double> outRearSteeringTorqueCmd;

        OutputPort<bool> outBit01;
        OutputPort<bool> outBit02;
        OutputPort<bool> outBit03;
        OutputPort<bool> outBit04;
        OutputPort<bool> outBit05;
        OutputPort<bool> outBit06;
        OutputPort<bool> outBit07;
        OutputPort<bool> outBit08;


    protected:
    };

}

#endif
