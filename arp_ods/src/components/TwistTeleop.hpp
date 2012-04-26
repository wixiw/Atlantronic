/*
 * TwistTeleop.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef TWISTTELEOP_HPP_
#define TWISTTELEOP_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>
#include <models/core>

#include <std_msgs/Bool.h>

namespace arp_ods
{

class TwistTeleop: public OdsTaskContext
{
    public:
        TwistTeleop(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        double propLinearGain;
        double propAngularGain;
        double propLinearAcc;
        double propAngularAcc;
        double propExpertLinearGain;
        double propExpertAngularGain;
        double propExpertLinearAcc;
        double propExpertAngularAcc;
        bool propFilter;

        double attrSpeedCmd;
        double attrOldSpeedCmd;
        double attrSpeedDirection;
        double attrAngularCmd;
        double attrOldAngularCmd;
        arp_math::Twist2D attrTwistCmdCdg;

        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;
        //commandes joystick
        RTT::InputPort<bool> inDeadMan;
        RTT::InputPort<double> inXSpeed;
        RTT::InputPort<double> inYSpeed;
        RTT::InputPort<double> inThetaSpeed;
        //quand le port est vrai on utilise une configuration musclée
        RTT::InputPort<bool> inExpertMode;
        //mesure actuelle
        RTT::InputPort<arp_model::UbiquityParams>inParams;
        RTT::InputPort<bool> inPower;
        RTT::InputPort<std_msgs::Bool> inBootUpDone;
        RTT::OperationCaller<bool(bool)> m_coSetMotorPower;

        void createOrocosItf();

};

} /* namespace arp_ods */
#endif /* TWISTTELEOP_HPP_ */
