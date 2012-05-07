/*
 * RosOdsItf.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "RosOdsItf.hpp"
#include <rtt/Component.hpp>

using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::RosOdsItf )

RosOdsItf::RosOdsItf(std::string const name):
        OdsTaskContext(name),
        m_actionServer("MotionControl", boost::bind(&RosOdsItf::newOrderCB, this, _1), false),
        propOrderConfig(),
        m_blockTime(0.0)
{
    createOrocosInterface();
    createRosInterface();

    m_order = orders::defaultOrder;

    //TODO a remplacer par la lecture des proprietes en XML ... ce qui necessite d'ecrire le typekit
    //WLA : j'ai tenté de le générer automatiquement ça n'a pas l'air d'avoir fonctionner,
    //c'est étonnant c'est pourtant une structure simple
    propOrderConfig.RADIUS_APPROACH_ZONE = 0.010;
    propOrderConfig.ANGLE_ACCURACY = 0.018;//1°
    propOrderConfig.DISTANCE_ACCURACY = 0.010;

    propOrderConfig.LIN_VEL_MAX = 1.0;
    propOrderConfig.ANG_VEL_MAX = 15.0;
    propOrderConfig.VEL_FINAL = 0.150;

    propOrderConfig.ORDER_TIMEOUT = 10.0;
    propOrderConfig.PASS_TIMEOUT = 1.0;

    propOrderConfig.LIN_DEC=4.0;
    propOrderConfig.ANG_DEC=8.0;

}

bool RosOdsItf::configureHook()
{
    bool res = OdsTaskContext::configureHook();
    res &= getOperation("MotionControl",      "ooSetOrder",  m_ooSetOrder);
    res &= getOperation("MotionControl",      "ooSetVMax",  m_ooSetVMax);
    //TODO implementer le hook qui va bien pour checker la propOrderConfig
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
    bool tmp;
    bool finished;

    //if a goal was already defined it is refused
    if (m_order->getType() != NO_ORDER)
    {
        ROS_ERROR(
                "MotionControl : you are trying to send a new order (%s) since the last one is not processed. It won't be performed, try to interrupt it before",
                goal->move_type.c_str());
        goto abort;
    }


    //TODO a remplacer par une factory plus efficace
    char string[250];

    if (goal->move_type == "OMNIDIRECT")
    {
        inPose.readNewest(pose);
        m_order = OmnidirectOrder::createOrder(goal, pose, propOrderConfig);
        sprintf( string, "new Omnidirect goal (%0.3f,%0.3f,%0.3f) pass %d", goal->x_des, goal->y_des,
                goal->theta_des, goal->passe);
        LOG(Info) << string << endlog();
    }
    else if (goal->move_type == "OPENLOOP")
    {
        inPose.readNewest(pose);
        m_order = OpenloopOrder::createOrder(goal, pose, propOrderConfig);
        sprintf( string, "new Openloop goal Twist:(%0.3f,%0.3f,%0.3f) time : %0.3f ", goal->x_speed, goal->y_speed,
                goal->theta_speed, goal->openloop_duration);
        LOG(Info) << string << endlog();
    }
    else if (goal->move_type == "REPLAY")
    {
        inPose.readNewest(pose);
        m_order = ReplayOrder::createOrder(goal, pose, propOrderConfig);
        sprintf( string, "new Replay goal Twist:(%0.3f,%0.3f,%0.3f) time : %0.3f ", goal->x_speed, goal->y_speed,
                goal->theta_speed, goal->openloop_duration);
        LOG(Info) << string << endlog();
    }
    else
    {
        LOG(Error) << "order " << goal->move_type.c_str() << "is not possible" << endlog();
        goto abort;
    }

    if( m_ooSetOrder(m_order) == false )
    {

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
            LOG(Error) << goal->move_type.c_str() << " Preempted" << endlog();
            //TODO faudrait peut être l'envoyer à LittleSexControl
            goto preempted;
        }

        //If order is in error something went wrong
        bool inError;
        if( inCurrentOrderIsInError.readNewest(inError) != NewData )
        {
            inError = false;
        }
        if ( inError )
        {
            LOG(Error) << goal->move_type.c_str() << ": not processed due to MODE_ERROR" << endlog();
            goto abort;
        }

        //robot is blocked ?
        bool blocked;
        double time=getTime();
        inRobotBlocked.readNewest(blocked);
        if(blocked and time-m_blockTime>1.0) //1 of occultation, to allow the beginning of the new motion
        {
            LOG(Error) << goal->move_type.c_str() << ": not processed due to Robot Blockage detection" << endlog();
            m_blockTime= time;
            goto abort;
        }

        r.sleep();

        if( inCurrentOrderIsFinished.readNewest(finished) != NewData )
        {
            finished = false;
        }
    }
    //if we are here it's because an order has been interrupt and the robot is halted
    goto success;

    abort:
        inPose.readNewest(pose);
        result.x_end = pose.x();
        result.y_end = pose.y();
        result.theta_end = pose.h();
        m_order = orders::defaultOrder;
        m_actionServer.setAborted(result);
        return;
    success:
        ROS_INFO("%s: Finished", goal->move_type.c_str());
        inPose.readNewest(pose);
        result.x_end = pose.x();
        result.y_end = pose.y();
        result.theta_end = pose.h();
        m_order = orders::defaultOrder;
        m_actionServer.setSucceeded(result);
        return;
    preempted:
        m_order = orders::defaultOrder;
        m_actionServer.setPreempted();
        return;
}

bool RosOdsItf::setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res)
{
    return m_ooSetVMax(req.vMax);;
}

void RosOdsItf::createOrocosInterface()
{
    addProperty("propOrderConfig",propOrderConfig);

    addPort("inPose",inPose);
    addPort("inCurrentOrderIsFinished",inCurrentOrderIsFinished);
    addPort("inCurrentOrderIsInError",inCurrentOrderIsInError);
    addPort("inRobotBlocked",inRobotBlocked);
}

void RosOdsItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvSetVMax = nh.advertiseService("/MotionControl/setVMax", &RosOdsItf::setVMaxCallback, this);
}
