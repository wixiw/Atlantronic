/*
 * RosRluItf.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ROSRLUITF_HPP_
#define ROSRLUITF_HPP_

#include "RluTaskContext.hpp"
#include <math/core>
#include <arp_core/Pose.h>

namespace arp_rlu
{

class RosRluItf: public arp_rlu::RluTaskContext
{
    public:
        RosRluItf(std::string const name);
        void updateHook();

    protected:
        RTT::InputPort<arp_math::EstimatedPose2D> inPose;
        RTT::InputPort<arp_math::EstimatedTwist2D> inTwist;
        RTT::OutputPort<arp_core::Pose> outPose;
};

} /* namespace arp_rlu */
#endif /* ROSRLUITF_HPP_ */
