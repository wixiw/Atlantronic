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

namespace arp_ods
{

class TwistTeleop: public OdsTaskContext
{
    public:
        TwistTeleop(const std::string& name);
        void updateHook();

    protected:
        double propLinearGain;
        double propAngularGain;
        double propLinearAcc;
        double propAngularAcc;
        bool propFilter;

        arp_math::Twist2D attrTwistCmdB4Filter;
        arp_math::Twist2D attrTwistCmdCdg;
        arp_math::EstimatedTwist2D attrInTwist;

        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;
        //commandes joystick
        RTT::InputPort<double> inXSpeed;
        RTT::InputPort<double> inYSpeed;
        RTT::InputPort<double> inThetaSpeed;
        //mesure actuelle
        RTT::InputPort<arp_math::EstimatedTwist2D> inTwist;
        RTT::InputPort<arp_model::UbiquityParams>inParams;
};

} /* namespace arp_ods */
#endif /* TWISTTELEOP_HPP_ */
