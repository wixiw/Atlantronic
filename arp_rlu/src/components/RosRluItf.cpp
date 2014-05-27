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
using namespace arp_time;
using namespace RTT;
using namespace std;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_rlu::RosRluItf )

RosRluItf::RosRluItf(std::string const name):
RluTaskContext(name)
{
    addPort("inPose",inPose);
    addPort("inLocalizationState", inLocalizationState);
    addPort("inLocalizationMode", inLocalizationMode);
    addPort("inLocalizationQuality", inLocalizationQuality);
    addPort("inLocalizationVisibility", inLocalizationVisibility);
    addPort("inICRSpeed",inICRSpeed);
    addPort("inOpponents",inOpponents);
    addPort("outPose",outPose);
    addPort("outSpeed",outSpeed);
    addPort("outOpponents",outOpponents);
    addPort("outLocalizationState",outLocalizationState);
    createRosInterface();
}

bool RosRluItf::configureHook()
{
    bool res = RluTaskContext::configureHook();

    res &= getOperation("Localizator",     "ooInitialize",                         m_ooInitialize);
    res &= getOperation("Localizator",     "ooSwitchToRedConfig",                  m_ooSwitchToRedConfig);
    res &= getOperation("Localizator",     "ooSwitchToYellowConfig",               m_ooSwitchToYellowConfig);




    return res;
}

void RosRluItf::updateHook()
{
    RluTaskContext::updateHook();
    EstimatedPose2D pIn;
    EstimatedTwist2D tIn;
    EstimatedICRSpeed speedIn;
    std::vector<arp_math::EstimatedPose2D> opponentsIn;

    inPose.readNewest(pIn);
    inICRSpeed.readNewest(speedIn);
    tIn = speedIn.twist();
    inOpponents.readNewest(opponentsIn);

    Pose pOut;
    pOut.x = pIn.x();
    pOut.y = pIn.y();
    pOut.theta = pIn.h();
    pOut.vx = tIn.vx();
    pOut.vy = tIn.vy();
    pOut.vtheta = tIn.vh();
    outPose.write(pOut);


    //speed
    ICRSpeedMsg sOut;
    sOut.ro = speedIn.ro();
    sOut.phi = speedIn.phi();
    sOut.delta = speedIn.delta();
    outSpeed.write(sOut);

    LocalizationState stateOut;
    int locState = -1; //0; //STOPPED
    inLocalizationState.readNewest(locState);
    stateOut.state = locState;
    int locMode = -1; //0; //ODO_ONLY
    inLocalizationMode.readNewest(locMode);
    stateOut.mode = locMode;
    int locQuality = -1; //0; //LOST
    inLocalizationQuality.readNewest(locQuality);
    stateOut.quality = locQuality;
    int locVisu = -1; //0; //NONE
    inLocalizationVisibility.readNewest(locVisu);
    stateOut.visibility = locVisu;
    outLocalizationState.write(stateOut);

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
    opponentsOut.date = getAbsoluteTime();
    outOpponents.write(opponentsOut);
}

bool RosRluItf::srvInitialize(SetPosition::Request& req, SetPosition::Response& res)
{
    res.success = m_ooInitialize(req.x,req.y,req.theta);
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
    if( req.color.compare("yellow") == 0  )
    {
        m_ooSwitchToYellowConfig();
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
    m_srvSetColor = nh.advertiseService("/Localizator/setColor", &RosRluItf::srvSetColor, this);
}
