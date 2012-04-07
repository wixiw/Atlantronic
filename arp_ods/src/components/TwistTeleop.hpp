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

        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;
        RTT::InputPort<double> inXSpeed;
        RTT::InputPort<double> inYSpeed;
        RTT::InputPort<double> inThetaSpeed;
        RTT::InputPort<arp_model::UbiquityParams>inParams;
};

} /* namespace arp_ods */
#endif /* TWISTTELEOP_HPP_ */
