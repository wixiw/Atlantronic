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
#include <arp_core/OpponentsList.h>
#include <arp_core/LocalizatorState.h>

//rossrv
#include <arp_core/SetPosition.h>
#include <arp_core/SetColor.h>
#include <arp_rlu/AutoInit.h>

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
        RTT::InputPort<int> inLocalizationState;
        RTT::InputPort<int> inLocalizationMode;
        RTT::InputPort<int> inLocalizationQuality;
        RTT::InputPort<int> inLocalizationVisibility;
        RTT::InputPort<arp_math::EstimatedTwist2D> inTwist;
        RTT::InputPort< std::vector<arp_math::EstimatedPose2D> > inOpponents;
        RTT::OutputPort<arp_core::Pose> outPose;
        RTT::OutputPort< arp_core::LocalizatorState > outLocalizatorState;
        RTT::OutputPort< arp_core::OpponentsList > outOpponents;

        RTT::OperationCaller<bool(double,double,double)> m_ooInitialize;
        RTT::OperationCaller<bool(void)> m_ooDo;
        RTT::OperationCaller<arp_math::EstimatedPose2D(void)> m_ooGetEstimatedPose;
        RTT::OperationCaller<double(void)> m_ooGetRelativeHeadingForConfirmation;
        RTT::OperationCaller<void(void)> m_ooSwitchToRedConfig;
        RTT::OperationCaller<void(void)> m_ooSwitchToPurpleConfig;

        bool srvInitialize(arp_core::SetPosition::Request& req, arp_core::SetPosition::Response& res);
        bool srvAutoInit(AutoInit::Request& req, AutoInit::Response& res);
        bool srvSetColor(arp_core::SetColor::Request& req, arp_core::SetColor::Response& res);
        void createRosInterface();

        /** node handles to store services **/
        ros::ServiceServer m_srvInitialize;
        ros::ServiceServer m_srvAutoInit;
        ros::ServiceServer m_srvSetColor;

};

} /* namespace arp_rlu */
#endif /* ROSRLUITF_HPP_ */
