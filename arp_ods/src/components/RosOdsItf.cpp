/*
 * RosOdsItf.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "RosOdsItf.hpp"
#include <rtt/Component.hpp>
#include "ods_logger/Logger.hpp"
#include "time/ArdTime.hpp"
#include <iostream>

using namespace arp_math;
using namespace arp_time;
using namespace arp_ods;
using namespace orders;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::RosOdsItf)

RosOdsItf::RosOdsItf(std::string const name) :
        OdsTaskContext(name), m_actionServer("MotionControl", boost::bind(&RosOdsItf::newOrderCB, this, _1), false), m_blockTime(
                0.0),OTG()
{

    createOrocosInterface();
    createRosInterface();

    m_order = OrderFactory::createDefaultOrder();

}

bool RosOdsItf::configureHook()
{
    bool res = OdsTaskContext::configureHook();
    res &= getOperation("MotionControl", "ooSetOrder", m_ooSetOrder);
    res &= getOperation("MotionControl", "ooSetVMax", m_ooSetVMax);
    return res;
}

bool RosOdsItf::startHook()
{
    bool res = OdsTaskContext::startHook();
    m_actionServer.start();
    return res;
}

void RosOdsItf::newOrderCB(const OrderGoalConstPtr &goal)
{
    ros::Rate r(20);
    OrderResult result;
    EstimatedPose2D pose;
    EstimatedICRSpeed speed;
    UbiquityParams params;
    bool tmp;
    bool finished;

    inPose.readNewest(pose);
    inSpeed.readNewest(speed);
    inParams.readNewest(params);

    arp_ods::Log(Info) << " humhum **********params.getMaxRobotSpeed() " << params.getMaxRobotSpeed();

    UbiquityMotionState currentMotionState(pose, speed);

    if (goal->move_type == "OMNIDIRECT2" or goal->move_type == "OPENLOOP" or goal->move_type == "REPLAY"
            or goal->move_type == "STAY")
    {
        m_order = OrderFactory::createOrder(goal, currentMotionState, params);
        m_order->setOTG(&OTG);
        arp_ods::Log(Info) << " New Order! " << goal->move_type << endlog();
    }
    else
    {
        arp_ods::Log(Error) << "order " << goal->move_type.c_str() << " is not possible" << endlog();
        goto abort;
    }

    if (m_ooSetOrder(m_order) == false)
    {
        arp_ods::Log(Error) << "order " << goal->move_type.c_str() << " failed to call m_ooSetOrder" << endlog();
        goto abort;
    }

    //TODO WLA mettre ça dans order
    //m_orderSelector.setWorkTimeout(15);

    finished = inCurrentOrderIsFinished.readNewest(tmp) == NewData && tmp;
    while (!finished)
    {
        //An interrupt has been requested
        if (m_actionServer.isPreemptRequested() || !ros::ok())
        {
            arp_ods::Log(Error) << goal->move_type.c_str() << " Preempted" << endlog();
            //TODO faudrait peut être l'envoyer à LittleSexControl
            goto preempted;
        }

        //If order is in error something went wrong
        bool inError;
        if (inCurrentOrderIsInError.readNewest(inError) != NewData)
        {
            inError = false;
        }
        if (inError)
        {
            arp_ods::Log(Error) << goal->move_type.c_str() << ": not processed due to MODE_ERROR" << endlog();
            goto abort;
        }

        //robot is blocked ?
        bool blocked;
        ArdAbsoluteTime time = getAbsoluteTime();
        inRobotBlocked.readNewest(blocked);
        if (blocked and time - m_blockTime > 1.0) //1 of occultation, to allow the beginning of the new motion
        {
            arp_ods::Log(Error) << goal->move_type.c_str() << ": not processed due to Robot Blockage detection"
                    << endlog();
            m_blockTime = time;
            goto abort;
        }

        r.sleep();

        if (inCurrentOrderIsFinished.readNewest(finished) != NewData)
        {
            finished = false;
        }
    }
    //if we are here it's because an order has been interrupt and the robot is halted
    goto success;

    abort: inPose.readNewest(pose);
    result.x_end = pose.x();
    result.y_end = pose.y();
    result.theta_end = pose.h();
    m_order = OrderFactory::createStayOrder(currentMotionState, params);
    m_ooSetOrder(m_order);
    m_actionServer.setAborted(result);
    return;
    success:
    inPose.readNewest(pose);
    result.x_end = pose.x();
    result.y_end = pose.y();
    result.theta_end = pose.h();
    m_actionServer.setSucceeded(result);
    return;
    preempted: m_order = OrderFactory::createStayOrder(currentMotionState, params);
    m_ooSetOrder(m_order);
    m_actionServer.setPreempted();
    return;
}

bool RosOdsItf::setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res)
{
    return m_ooSetVMax(req.vMax);
}

void RosOdsItf::createOrocosInterface()
{
    addPort("inPose", inPose);
    addPort("inSpeed", inSpeed);
    addPort("inParams", inParams);
    addPort("inCurrentOrderIsFinished", inCurrentOrderIsFinished);
    addPort("inCurrentOrderIsInError", inCurrentOrderIsInError);
    addPort("inRobotBlocked", inRobotBlocked);

}

void RosOdsItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvSetVMax = nh.advertiseService("/MotionControl/setVMax", &RosOdsItf::setVMaxCallback, this);
}

