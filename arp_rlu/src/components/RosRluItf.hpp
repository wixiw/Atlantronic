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

using namespace arp_math;
using namespace arp_core;

namespace arp_rlu
{

class RosRluItf: public arp_rlu::RluTaskContext
{
    public:
        RosRluItf(std::string const name);
        void updateHook();

    protected:
        InputPort<EstimatedPose2D> inPose;
        InputPort<EstimatedTwist2D> inTwist;
        OutputPort<Pose> outPose;
};

} /* namespace arp_rlu */
#endif /* ROSRLUITF_HPP_ */
