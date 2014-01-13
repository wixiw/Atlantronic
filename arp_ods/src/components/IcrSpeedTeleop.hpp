/*
 * IcrSpeedTeleop.hpp
 *
 *  Created on: Jan 12, 2014
 *      Author: willy
 */

#ifndef ICRSPEEDTELEOP_HPP_
#define ICRSPEEDTELEOP_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>
#include <models/core>

#include <std_msgs/Bool.h>

namespace arp_ods
{

class IcrSpeedTeleop: public OdsTaskContext
{
    public:
        IcrSpeedTeleop(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        double propLinearGain;
        double propLinearAcc;
        double propExpertLinearGain;
        double propExpertLinearAcc;
        bool propFilter;

        double attrOldRho;

        double attrRho;
        double attrPhi;
        double attrDelta;

        arp_math::ICRSpeed attrVelocityCmdCdg;

        RTT::OutputPort<arp_math::ICRSpeed> outICRSpeedCmd;
        //commandes joystick
        RTT::InputPort<bool> inDeadMan;
        RTT::InputPort<double> inRoSpeedCmd;
        RTT::InputPort<double> inPhiCmd;
        RTT::InputPort<double> inDeltaCmd;
        //quand le port est vrai on utilise une configuration musclée
        RTT::InputPort<bool> inExpertMode;
        //mesure actuelle
        RTT::InputPort<arp_model::UbiquityParams>inParams;
        RTT::InputPort<bool> inPower;
        RTT::InputPort<std_msgs::Bool> inBootUpDone;
        RTT::OperationCaller<bool(bool)> m_coSetMotorPower;

        void createOrocosItf();

        /*
         * time of last call
         */
        double m_oldTime;
};

} /* namespace arp_ods */
#endif /* ICRSPEEDTELEOP_HPP_ */
