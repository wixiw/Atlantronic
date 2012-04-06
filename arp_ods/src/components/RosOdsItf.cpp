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

ORO_LIST_COMPONENT_TYPE( arp_ods::RosOdsItf )

RosOdsItf::RosOdsItf(std::string const name):
        OdsTaskContext(name),
        m_actionServer(ros::this_node::getName(), boost::bind(&RosOdsItf::newOrderCB, this, _1), false)
{
    addPort("inPose",inPose);
    addPort("inCurrentOrderIsFinished",inCurrentOrderIsFinished);
    addPort("inCurrentOrderIsInError",inCurrentOrderIsInError);
    addPort("outOrder",outOrder);

    updateParams();
    m_order = order::defaultOrder;
    m_actionServer.start();
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
        m_order = FantomOrder::createOrder(goal, pose, m_orderConfig);
        ROS_INFO("MotionControl : new Fantom goal (%0.3f,%0.3f,%0.3f) reverse : %d pass %d", goal->x_des, goal->y_des,
                goal->theta_des, goal->reverse, goal->passe);
    }
    else if (goal->move_type == "CAP")
    {
        inPose.readNewest(pose);
        m_order = RotationOrder::createOrder(goal, pose, m_orderConfig);
        ROS_INFO("MotionControl : new Rotation goal (cap=%0.3f)", goal->theta_des);
    }
    else
    {
        ROS_ERROR("%s: not possible", goal->move_type.c_str());
        goto abort;
    }

    //TODO envoyer l'ordre !
    //ooSetOrder(m_order);


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
        inCurrentOrderIsInError.readNewest(inError);
        if ( inError )
        {
            ROS_ERROR("%s: not processed due to MODE_ERROR", goal->move_type.c_str());
            goto abort;
        }

        r.sleep();
        inCurrentOrderIsFinished.readNewest(finished);
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

void RosOdsItf::updateParams()
{
    if (ros::param::get("/arp_ods/LIN_VEL_MAX", m_orderConfig.LIN_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre LIN_VEL_MAX");

    if (ros::param::get("/arp_ods/ANG_VEL_MAX", m_orderConfig.ANG_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANG_VEL_MAX");

    if (ros::param::get("/MotionControl/ROTATION_GAIN", m_orderConfig.ROTATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_GAIN");

    if (ros::param::get("/MotionControl/ROTATION_D_GAIN", m_orderConfig.ROTATION_D_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_D_GAIN");

    if (ros::param::get("/MotionControl/TRANSLATION_GAIN", m_orderConfig.TRANSLATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre TRANSLATION_GAIN");

    if (ros::param::get("/MotionControl/VEL_FINAL", m_orderConfig.VEL_FINAL) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre VEL_FINAL");

    if (ros::param::get("/MotionControl/FANTOM_COEF", m_orderConfig.FANTOM_COEF) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre FANTOM_COEF");

    if (ros::param::get("/MotionControl/RADIUS_APPROACH_ZONE", m_orderConfig.RADIUS_APPROACH_ZONE) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RADIUS_APPROACH_ZONE");

    if (ros::param::get("/MotionControl/DISTANCE_ACCURACY", m_orderConfig.DISTANCE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre DISTANCE_ACCURACY");

    if (ros::param::get("/MotionControl/ANGLE_ACCURACY", m_orderConfig.ANGLE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANGLE_ACCURACY");

    if (ros::param::get("/MotionControl/PASS_TIMEOUT", m_orderConfig.PASS_TIMEOUT) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre PASS_TIMEOUT");

    if (ros::param::get("/MotionControl/ORDER_TIMEOUT", m_orderConfig.ORDER_TIMEOUT) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ORDER_TIMEOUT");

    //TODO
//    if (ros::param::get("/MotionControl/WHEEL_BLOCKED_TIMEOUT", WHEEL_BLOCKED_TIMEOUT) == 0)
//        ROS_FATAL("pas reussi a recuperer le parametre WHEEL_BLOCKED_TIMEOUT");

}


bool RosOdsItf::setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res)
{
    //TODO a remplacer par l'appel d'une operation Orocos
//    if (req.setToDefault == false)
//    {
//        m_vMax = req.vMax;
//    }
//    else
//    {
//        m_vMax = m_orderConfig.LIN_VEL_MAX;
//    }

    return false;
}

