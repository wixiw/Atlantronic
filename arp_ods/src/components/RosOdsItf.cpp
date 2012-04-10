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
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::RosOdsItf )

RosOdsItf::RosOdsItf(std::string const name):
        OdsTaskContext(name),
        m_actionServer("MotionControl", boost::bind(&RosOdsItf::newOrderCB, this, _1), false),
        propOrderConfig()
{
    createOrocosInterface();
    createRosInterface();

    m_order = order::defaultOrder;

    //TODO a remplacer par la lecture des proprietes en XML ... ce qui necessite d'ecrire le typekit
    //WLA : j'ai tenté de le générer automatiquement ça n'a pas l'air d'avoir fonctionner,
    //c'est étonnant c'est pourtant une structure simple
    propOrderConfig.RADIUS_APPROACH_ZONE = 0.200; //20cm
    propOrderConfig.ANGLE_ACCURACY = 0.087;//5°
    propOrderConfig.DISTANCE_ACCURACY = 0.020;//20mm

    propOrderConfig.LIN_VEL_MAX = 1; //1m/s
    propOrderConfig.ANG_VEL_MAX = 3; //3 rad/s
    propOrderConfig.VEL_FINAL = 0.300;//300mm/s

    propOrderConfig.ORDER_TIMEOUT = 5; //5s
    propOrderConfig.PASS_TIMEOUT = 1; //1s

    propOrderConfig.FANTOM_COEF = 1;
    propOrderConfig.ROTATION_D_GAIN = 0;
    propOrderConfig.ROTATION_GAIN = 1;
    propOrderConfig.TRANSLATION_GAIN = 1;
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

    //if a goal was already define it is refused
    if (m_order->getType() != NO_ORDER)
    {
        ROS_ERROR(
                "MotionControl : you are trying to send a new order (%s) since the last one is not processed. It won't be performed, try to interrupt it before",
                goal->move_type.c_str());
        goto abort;
    }

    //annulation du blocage avant de partir pour ne pas s'en repayer un
    //TODO a voir comment on met ca en 2012
//    m_blockTime = 0;
//    m_wheelBlockedTimeout = false;

    //TODO a remplacer par une factory plus efficace
    if (goal->move_type == "POINTCAP")
    {
        inPose.readNewest(pose);
        m_order = FantomOrder::createOrder(goal, pose, propOrderConfig);
        ROS_INFO("MotionControl : new Fantom goal (%0.3f,%0.3f,%0.3f) reverse : %d pass %d", goal->x_des, goal->y_des,
                goal->theta_des, goal->reverse, goal->passe);
    }
    else if (goal->move_type == "CAP")
    {
        inPose.readNewest(pose);
        m_order = RotationOrder::createOrder(goal, pose, propOrderConfig);
        ROS_INFO("MotionControl : new Rotation goal (cap=%0.3f)", goal->theta_des);
    }
    else if (goal->move_type == "OMNIDIRECT")
    {
        inPose.readNewest(pose);
        m_order = OmnidirectOrder::createOrder(goal, pose, propOrderConfig);
        ROS_INFO("MotionControl : new Omnidirect goal (%0.3f,%0.3f,%0.3f) ", goal->x_des, goal->y_des,
                goal->theta_des);
    }
    else
    {
        ROS_ERROR("%s: not possible", goal->move_type.c_str());
        goto abort;
    }

    m_ooSetOrder(m_order);


    //TODO WLA mettre ça dans order
    //m_orderSelector.setWorkTimeout(15);


    bool finished;
    inCurrentOrderIsFinished.readNewest(finished);
    while (!finished)
    {
        //An interrupt has been requested
        if (m_actionServer.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", goal->move_type.c_str());
            goto preempted;
        }

        //TODO a voir comment on met ca en 2012
//        //Blocage
//        if( m_wheelBlockedTimeout )
//        {
//            ROS_INFO("%s: WheelBlocked, canceling move", goal->move_type.c_str());
//            goto abort;
//        }

        //If order is in error something went wrong
        bool inError;
        if( inCurrentOrderIsInError.readNewest(inError) != NewData )
        {
            inError = false;
        }
        if ( inError )
        {
            ROS_ERROR("%s: not processed due to MODE_ERROR", goal->move_type.c_str());
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
        m_order = order::defaultOrder;
        m_actionServer.setAborted(result);
        return;
    success:
        ROS_INFO("%s: Finished", goal->move_type.c_str());
        inPose.readNewest(pose);
        result.x_end = pose.x();
        result.y_end = pose.y();
        result.theta_end = pose.h();
        m_order = order::defaultOrder;
        m_actionServer.setSucceeded(result);
        return;
    preempted:
        m_order = order::defaultOrder;
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
    addPort("outOrder",outOrder);
}

void RosOdsItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvSetVMax = nh.advertiseService("/MotionControl/setVMax", &RosOdsItf::setVMaxCallback, this);
}
