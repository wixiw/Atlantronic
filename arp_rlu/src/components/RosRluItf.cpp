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
    addPort("inOpponents",inOpponents);
    addPort("outPose",outPose);
    addPort("outOpponents",outOpponents);
    createRosInterface();
}

bool RosRluItf::configureHook()
{
    bool res = RluTaskContext::configureHook();

    res &= getOperation("Localizator",              "ooInitialize",                         m_ooInitialize);
    res &= getOperation("LaserOnlyLocalizator",     "ooDo",                                 m_ooDo);
    res &= getOperation("LaserOnlyLocalizator",     "ooGetEstimatedPose",                   m_ooGetEstimatedPose);
    res &= getOperation("LaserOnlyLocalizator",     "ooGetRelativeHeadingForConfirmation",  m_ooGetRelativeHeadingForConfirmation);
    res &= getOperation("LaserOnlyLocalizator",     "ooSwitchToRedConfig",                  m_ooSwitchToRedConfig);
    res &= getOperation("LaserOnlyLocalizator",     "ooSwitchToPurpleConfig",               m_ooSwitchToPurpleConfig);




    return res;
}

void RosRluItf::updateHook()
{
    RluTaskContext::updateHook();
    EstimatedPose2D pIn;
    EstimatedTwist2D tIn;
    std::vector<arp_math::EstimatedPose2D> opponentsIn;

    inPose.readNewest(pIn);
    inTwist.readNewest(tIn);
    inOpponents.readNewest(opponentsIn);

    Pose pOut;
    pOut.x = pIn.x();
    pOut.y = pIn.y();
    pOut.theta = pIn.h();
    pOut.vx = tIn.vx();
    pOut.vy = tIn.vy();
    pOut.vtheta = tIn.vh();
    outPose.write(pOut);

    arp_core::OpponentsList opponentsOut;
    std::vector<arp_math::EstimatedPose2D>::iterator opp;
    arp_core::Pose pose;
    for( opp = opponentsIn.begin(); opp != opponentsIn.end() ;  opp++ )
    {
        EstimatedPose2D pose2d = (*opp);
        pose.x = pose2d.x();
        pose.y = pose2d.y();
        pose.theta = pose2d.h();
        opponentsOut.Opponents.push_back(pose);
    }
    opponentsOut.nbOpponents = opponentsIn.size();
    opponentsOut.date = timespec2Double(attrUpdateTime);
    outOpponents.write(opponentsOut);
}

bool RosRluItf::srvInitialize(SetPosition::Request& req, SetPosition::Response& res)
{
    res.success = m_ooInitialize(req.x,req.y,req.theta);
    return res.success;
}

bool RosRluItf::srvAutoInit(AutoInit::Request& req, AutoInit::Response& res)
{
    res.success = m_ooDo();

    if( res.success)
    {
        arp_math::EstimatedPose2D pose;
        pose = m_ooGetEstimatedPose();
        LOG(Info) << "RosRluItf : srvAutoInit, ooDo succeed with " << pose.toString() << endlog();

        res.success &= m_ooInitialize(pose.x(),pose.y(),pose.h());
        if( !res.success)
        {
            LOG(Error) << "RosRluItf : m_ooInitialize failed to set " << pose.toString() << endlog();
        }
    }
    else
    {
        double heading = m_ooGetRelativeHeadingForConfirmation();
        LOG(Info) << "RosRluItf : srvAutoInit, ooDo failed and suggest heading of " << heading << endlog();
        res.h_retry = heading;
    }

    return res.success;
}

bool RosRluItf::srvSetColor(SetColor::Request& req, SetColor::Response& res)
{
    if( req.color.compare("red") == 0  )
    {
        m_ooSwitchToRedConfig();
        res.success = true;
        return res.success;
    }
    if( req.color.compare("purple") == 0  )
    {
        m_ooSwitchToPurpleConfig();
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}



void RosRluItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvInitialize = nh.advertiseService("/Localizator/setPosition", &RosRluItf::srvInitialize, this);
    m_srvAutoInit = nh.advertiseService("/Localizator/setAutoInit", &RosRluItf::srvAutoInit, this);
    m_srvSetColor = nh.advertiseService("/Localizator/setColor", &RosRluItf::srvSetColor, this);
}
