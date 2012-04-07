/*
 * RosRluItf.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "RosRluItf.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;
using namespace std;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_rlu::RosRluItf )

RosRluItf::RosRluItf(std::string const name):
        RluTaskContext(name)
{
    addPort("inPose",inPose);
    addPort("inTwist",inTwist);
    addPort("outPose",outPose);

}

void RosRluItf::updateHook()
{
    RluTaskContext::updateHook();
    EstimatedPose2D pIn;
    EstimatedTwist2D tIn;
    inPose.readNewest(pIn);
    inTwist.readNewest(tIn);

    Pose pOut;
    pOut.x = pIn.x();
    pOut.y = pIn.y();
    pOut.theta = pIn.h();
    pOut.vx = tIn.vx();
    pOut.vy = tIn.vy();
    pOut.vtheta = tIn.vh();
    outPose.write(pOut);
}
