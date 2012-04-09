/*
 * RosRluItf.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ROSRLUITF_HPP_
#define ROSRLUITF_HPP_

#include "RluTaskContext.hpp"
#include <ros/ros.h>
#include <math/core>
//rosmsg
#include <arp_core/Pose.h>
//rossrv
#include <arp_core/SetPosition.h>

namespace arp_rlu
{

class RosRluItf: public arp_rlu::RluTaskContext
{
    public:
        RosRluItf(std::string const name);
        bool configureHook();
        void updateHook();

    protected:
        RTT::InputPort<arp_math::EstimatedPose2D> inPose;
        RTT::InputPort<arp_math::EstimatedTwist2D> inTwist;
        RTT::OutputPort<arp_core::Pose> outPose;

        RTT::OperationCaller<bool(double,double,double)> m_ooInitialize;

        bool srvInitialize(arp_core::SetPosition::Request& req, arp_core::SetPosition::Response& res);
        void createRosInterface();

        /** node handle to store the service advertiser srvInitialize**/
        ros::ServiceServer m_srvInitialize;

};

} /* namespace arp_rlu */
#endif /* ROSRLUITF_HPP_ */
