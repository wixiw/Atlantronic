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
#include "control/planners/OnlineSpeedGenerator.hpp"
#include "control/planners/PosVelAcc.hpp"
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
        double propMaxRho;
        double propMaxRhoAcc;
        double propExpertMaxRho;
        double propExpertMaxRhoAcc;

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

        RTT::InputPort<bool> inRotationMode;

        //quand le port est vrai on utilise une configuration musclée
        RTT::InputPort<bool> inExpertMode;

        //mesure actuelle
        RTT::InputPort<arp_model::UbiquityParams>inParams;
        RTT::InputPort<bool> inPower;
        RTT::InputPort<std_msgs::Bool> inBootUpDone;
        RTT::OperationCaller<bool(bool)> m_coSetMotorPower;

        OnlineSpeedGenerator m_ovg;
        arp_ods::PosVelAcc m_state;

        void createOrocosItf();
};

} /* namespace arp_ods */
#endif /* ICRSPEEDTELEOP_HPP_ */
