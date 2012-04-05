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

using namespace arp_math;

namespace arp_ods
{

class TwistTeleop: public OdsTaskContext
{
    public:
        TwistTeleop(const std::string& name);
        void updateHook();

    protected:
        OutputPort<Twist2D> outTwistCmd;
        InputPort<double> inXSpeed;
        InputPort<double> inYSpeed;
        InputPort<double> inThetaSpeed;
};

} /* namespace arp_ods */
#endif /* TWISTTELEOP_HPP_ */
