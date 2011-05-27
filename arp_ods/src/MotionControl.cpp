#include <tf/transform_datatypes.h>
#include "MotionControl.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace nav_msgs;
using namespace geometry_msgs;

MotionControl::MotionControl() :
    m_order(order::defaultOrder), m_poseFromCallback(), m_currentPose(), m_lastPose(),
            m_actionServer(ros::this_node::getName(), boost::bind(&MotionControl::newOrderCB, this, _1), false)
{
    updateParams();

    ros::NodeHandle nodeHandle = ros::NodeHandle();

    // Suscribers
    pose_sub_ = nodeHandle.subscribe("Localizator/odomRos", 1, &MotionControl::poseCallback, this);
    //TODO WLA test laserator
    //pose_sub_WLA = nh_.subscribe("pose2D", 1,
    //        &MotionControl::pose2DCallback, this);

    vel_pub_ = nodeHandle.advertise<arp_core::Velocity> ("Command/velocity", 1);

    timerreport_srv = nodeHandle.advertiseService( ros::this_node::getName() + "/timerReport",
                &MotionControl::timerreportCallback, this);

    m_actionServer.start();

}

void MotionControl::getInputs()
{
    m_lastPose = m_currentPose;
    m_currentPose.x = m_poseFromCallback.pose.pose.position.x;
    m_currentPose.y = m_poseFromCallback.pose.pose.position.y;
    geometry_msgs::Quaternion thetaQuaternion = m_poseFromCallback.pose.pose.orientation;
    m_currentPose.theta = tf::getYaw(thetaQuaternion);
    m_currentPose.date = m_poseFromCallback.header.stamp.toSec();

    //ROS_INFO("execute : m_position : x=%.3f, y=%.3f, theta=%.3f", m_position.x(), m_position.y(), m_cap.angle());
    //ROS_INFO("MOTION: %.3f", m_currentPose.date - m_lastPose.date);
}

/////////////////////////// MAIN LOOP//////////////////////
//this is the main loop of the motion control.
void MotionControl::execute()
{
    // timer to scope performances
    timer_.Start();

    //read inputs from callbacks
    getInputs();

    m_orderMutex.lock();

    //compute current order mode
    m_order->switchMode(m_currentPose);

    // calcule les consignes
    m_computedVelocityCmd = m_order->computeSpeed(m_currentPose);

    m_orderMutex.unlock();

    //publish computed value
    setOutputs();

    // timer to scope performances
    timer_.Stop();
}

void MotionControl::setOutputs()
{
    //saturation des consignes
    m_computedVelocityCmd.linear = saturate(m_computedVelocityCmd.linear, -LIN_VEL_MAX, LIN_VEL_MAX);
    m_computedVelocityCmd.angular = saturate(m_computedVelocityCmd.angular, -ANG_VEL_MAX, ANG_VEL_MAX);

    //ROS_WARN("linear=%0.3f angular=%0.3f",m_computedVelocityCmd.linear,m_computedVelocityCmd.angular);

    // On publie la consigne
    vel_pub_.publish(m_computedVelocityCmd);
}

void MotionControl::poseCallback(OdometryConstPtr c)
{
    m_poseFromCallback = *c;
    //à chaque fois qu'on reçoit une position on fait un calcul
    execute();
}

// TODO WLA test laserator
/*
 void MotionControl::pose2DCallback(Pose2DConstPtr c)
 {
 ROS_WARN("Pose 2D ???");
 m_position = Vector2(c->x, c->y);
 m_cap = Rotation2(c->theta);
 }
 */

void MotionControl::newOrderCB(const OrderGoalConstPtr &goal)
{
    //TODO WLA : ajouter un log pour afficher l'ordre qu'on va executer en détail

    ROS_WARN("newOrderCB");
    ros::Rate r(20);
    OrderResult result;

    //if a goal was already define it is refused
    if (m_order->getType() != NO_ORDER)
    {
        ROS_ERROR(
                "MotionControl : you are trying to send a new order (%s) since the last one is not processed. It won't be performed, try to interrupt it before",
                goal->move_type.c_str());
        goto not_processed;
    }

    //prise de la main sur la mutex pour tripoter l'order
    m_orderMutex.lock();

    if (goal->move_type == "POINTCAP")
    {
        m_order = FantomOrder::createOrder(goal, m_currentPose);
        ROS_INFO("MotionControl : new Fantom goal (%0.3f,%0.3f,%0.3f) reverse : %d pass %d", goal->x_des, goal->y_des,
                goal->theta_des, goal->reverse, goal->passe);
        ROS_INFO("MotionControl : new Fantom order (%0.3f,%0.3f,%0.3f)", m_order->getEndPose().x, m_order->getEndPose().y,
                m_order->getEndPose().theta);
    }
    else if (goal->move_type == "CAP")
    {
        m_order = RotationOrder::createOrder(goal, m_currentPose);
        ROS_INFO("MotionControl : new Rotation goal (cap=%0.3f)", goal->theta_des);
    }
    else
    {
        ROS_ERROR("%s: not possible", goal->move_type.c_str());
        goto not_processed;
    }

    //TODO mapper les coefs fantômes
    m_order->setAngleAccuracy(ANGLE_ACCURACY);
    m_order->setDistanceAccurancy(DISTANCE_ACCURACY);
    m_order->setRadiusApproachZone(RADIUS_APPROACH_ZONE);
    m_order->setRadiusInitZone(0.0);
    //TODO WLA mettre des ros param
    //TODO WLA mettre ça dans order
    //m_orderSelector.setWorkTimeout(15);


    //restitution de la main
    m_orderMutex.unlock();

    while (m_order->getMode() != MODE_DONE)
    {
        //An interrupt has been requested
        if (m_actionServer.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", goal->move_type.c_str());
            goto preempted;
        }

        //If order is in error something went wrong
        if (m_order->getMode() == MODE_ERROR)
        {
            ROS_ERROR("%s: not processed due to MODE_ERROR", goal->move_type.c_str());
            goto not_processed;
        }

        r.sleep();
    }
    //if we are here it's because an order has been interrupt and the robot is halted
    goto success;

    not_processed: result.x_end = m_currentPose.x;
    result.y_end = m_currentPose.y;
    result.theta_end = m_currentPose.theta;
    m_order = order::defaultOrder;
    m_actionServer.setAborted(result);
    return;
    success: result.x_end = m_currentPose.x;
    result.y_end = m_currentPose.y;
    result.theta_end = m_currentPose.theta;
    m_order = order::defaultOrder;
    m_actionServer.setSucceeded(result);
    return;
    preempted: m_order = order::defaultOrder;
    m_actionServer.setPreempted();
    return;
}

void MotionControl::updateParams()
{
    if (ros::param::get("/arp_ods/DISTANCE_ACCURACY", DISTANCE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre DISTANCE_ACCURACY");

    if (ros::param::get("/arp_ods/ANGLE_ACCURACY", ANGLE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANGLE_ACCURACY");

    if (ros::param::get("/arp_ods/ROTATION_GAIN", ROTATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_GAIN");

    if (ros::param::get("/arp_ods/ROTATION_D_GAIN", ROTATION_D_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_D_GAIN");

    if (ros::param::get("/arp_ods/TRANSLATION_GAIN", TRANSLATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre TRANSLATION_GAIN");

    if (ros::param::get("/arp_ods/LIN_VEL_MAX", LIN_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre LIN_VEL_MAX");

    if (ros::param::get("/arp_ods/ANG_VEL_MAX", ANG_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANG_VEL_MAX");

    if (ros::param::get("/arp_ods/VEL_FINAL", VEL_FINAL) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre VEL_FINAL");

    if (ros::param::get("/arp_ods/RADIUS_APPROACH_ZONE", RADIUS_APPROACH_ZONE) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RADIUS_APPROACH_ZONE");

    if (ros::param::get("/arp_ods/FANTOM_COEF", FANTOM_COEF) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre FANTOM_COEF");

}

 bool MotionControl::timerreportCallback(TimerReport::Request& req, TimerReport::Response& res)
 {
     std::stringstream info ;
     info << "==============================================" << std::endl;
     info << ros::this_node::getName() << " Performance Report (ms)" << std::endl;
     info << "----------------------------------------------" << std::endl;
     info << "  [*] Number of samples used : " << timer_.GetRawRefreshTime().size() << std::endl;
     info << "  [*] Actual loop period   : mean=" << timer_.GetMeanRefreshTime() * 1000.0;
     info << "  , stddev=" << timer_.GetStdDevRefreshTime() * 1000.0;
     info << "  , min=" << timer_.GetMinRefreshTime() * 1000.0;
     info << "  , max=" << timer_.GetMaxRefreshTime() * 1000.0;
     info << "  , last=" << timer_.GetLastRefreshTime() * 1000.0 << std::endl;
     /*info << "  [*] Raw actual loop periods :  ( ";
     for(std::vector<double>::const_iterator it = timer_.GetRawRefreshTime().begin(); it != timer_.GetRawRefreshTime().end(); ++it)
       info << (*it) * 1000.0 << " ";
     info << " )" << std::endl;*/
     info << "  [*] Loop duration    : mean=" << timer_.GetMeanElapsedTime() * 1000.0;
     info << "  , stddev=" << timer_.GetStdDevElapsedTime() * 1000.0;
     info << "  , min=" << timer_.GetMinElapsedTime() * 1000.0;
     info << "  , max=" << timer_.GetMaxElapsedTime() * 1000.0;
     info << "  , last=" << timer_.GetLastElapsedTime() * 1000.0 << std::endl;
     /*info << "  [*] Raw loop durations :  ( ";
     for(std::vector<double>::const_iterator it = timer_.GetRawElapsedTime().begin(); it != timer_.GetRawElapsedTime().end(); ++it)
       info << (*it) * 1000.0 << " ";
     info << " )" << std::endl; */

     res.report = info.str();
     return true;
 }

