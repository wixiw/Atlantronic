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
    arp_core::Pose beginPose;
    arp_core::Pose endPose;
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
        endPose.x = goal->x_des;
        endPose.y = goal->y_des;
        endPose.theta = goal->theta_des;

        shared_ptr<FantomOrder> order_tmp(new FantomOrder());
        order_tmp->setFANTOM_COEF(FANTOM_COEF);
        order_tmp->setTRANSLATION_GAIN(TRANSLATION_GAIN);
        order_tmp->setROTATION_GAIN(ROTATION_GAIN);
        order_tmp->setROTATION_D_GAIN(ROTATION_D_GAIN);
        order_tmp->setVEL_FINAL(VEL_FINAL);
        m_order = static_cast<shared_ptr<MotionOrder> > (order_tmp);

        ROS_INFO("MotionControl : A new order (%s) is waiting", goal->move_type.c_str());
    }
    else if (goal->move_type == "CAP" and goal->passe == false)
    {
        endPose.theta = goal->theta_des;

        m_order = static_cast<shared_ptr<MotionOrder> > (shared_ptr<RotationOrder> (new RotationOrder()));

        ROS_INFO("MotionControl : A new order (%s) is waiting", goal->move_type.c_str());
    }
    else
    {
        ROS_ERROR("%s: not possible", goal->move_type.c_str());
        goto not_processed;
    }
    m_order->setBeginPose(beginPose);
    m_order->setEndPose(endPose);
    m_order->setReverse(goal->reverse);
    m_order->setPass(goal->passe);
    m_order->resetMode();
    m_order->setAngleAccuracy(ANGLE_ACCURACY);
    m_order->setDistanceAccurancy(DISTANCE_ACCURACY);
    m_order->setRadiusApproachZone(RADIUS_APPROACH_ZONE);
    m_order->setRadiusInitZone(0.0);
    //TODO WLA mettre des ros param
    //TODO WLA mettre ça dans order
    //m_orderSelector.setWorkTimeout(15);
    //m_orderSelector.setHaltLinearVelocity(0.010);
    //m_orderSelector.setHaltAngularVelocity(0.2);


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

/*
 void MotionControl::processMotion()
 {
 if (m_order.getModeSelector().getMode() == MODE_PASS)
 {
 return;
 }

 if (m_order.getModeSelector().getMode() == MODE_DONE)
 {
 m_linearSpeedCmd = 0;
 m_angularSpeedCmd = 0;
 return;
 }

 //permet de retourner le repere du robot pour le faire partir en marche arriere
 if (reverse == false)
 {
 orientLocal_ = m_cap;
 orient_desLocal_ = orient_des;
 sens_lin = 1;
 }
 else
 {
 orientLocal_ = Rotation2(normalizeAngle(m_cap.angle() + PI));
 orient_desLocal_ = Rotation2(normalizeAngle(orient_des.angle() + PI));
 sens_lin = -1;
 }

 //ROS_INFO("*************************************************************");
 //ROS_INFO("m_position : x=%.3f, y=%.3f, theta=%.3f", m_position.x(), m_position.y(), m_cap.angle());

 ///////////////////////////////////////////////// DISTANCES
 // calcul de la distance au point desire
 distance_to_despoint = (trans_des - m_position).norm();

 //calcul de la distance à la droite passant par le point desire et perpendiculaire à l'angle desire
 //la distance à la ligne d'arrivée en quelque sorte
 double distance_to_desline = (orient_desLocal_.inverse() * (trans_des - m_position)).x();

 // selon le mode, je regarde la distance au point ou a la droite
 if (m_order.getModeSelector().getMode() == MODE_RUN)
 distance_error = distance_to_despoint;
 if (m_order.getModeSelector().getMode() == MODE_APPROACH)
 distance_error = distance_to_desline;

 ///////////////////////////////////////////////// ANGLES
 //creation du point fantome
 //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
 //c'est lui qu'on va viser en cap
 Vector2 phantompoint = trans_des - FANTOM_COEF * distance_error * Vector2(cos(orient_desLocal_.angle()),
 sin(orient_desLocal_.angle()));
 //calcul de l'angle au point fantome
 Vector2 phantom = phantompoint - m_position;
 Vector2 direction_in_current = orientLocal_.inverse() * phantom;
 double angle_error_fant = atan2(direction_in_current.y(), direction_in_current.x());

 //calcul de l'erreur d'angle par rapport a l'objectif
 double angle_error_des = normalizeAngle(orient_desLocal_.angle() - orientLocal_.angle());

 if (m_order.getModeSelector().getMode() == MODE_RUN)
 angle_error = angle_error_fant;
 if (m_order.getModeSelector().getMode() == MODE_APPROACH)
 angle_error = angle_error_des;
 //////////////////////////////////////CONSIGNES

 // calcul de la derivee de l'angle_error
 double old_loop_date = loop_date;
 loop_date = ros::Time::now().toSec();
 double delta_date = loop_date - old_loop_date;

 double d_angle_error;
 if (old_loop_date != 0 && delta_date > 0)
 {
 d_angle_error = (angle_error - old_angle_error) / delta_date;
 }
 else
 {
 d_angle_error = 0;
 }
 old_angle_error = angle_error;

 //creation des consignes full patates
 double lin_vel_cons_full =
 (TRANSLATION_GAIN * sqrt2(distance_error) * linearReductionCoef(angle_error) + VEL_FINAL) * sens_lin;
 double ang_vel_cons_full = ROTATION_GAIN * angle_error;

 //saturation des consignes
 m_linearSpeedCmd = saturate(lin_vel_cons_full, -LIN_VEL_MAX, LIN_VEL_MAX);
 m_angularSpeedCmd = saturate(ang_vel_cons_full, -ANG_VEL_MAX, ANG_VEL_MAX);

 }
 */

/*
 void MotionControl::switchState()
 {
 //approaching ??
 if (mode == MODE_RUN)
 {
 if (distance_to_despoint < RADIUS_APPROACH_ZONE and passe == false)
 {
 //ROS_INFO("ligne %i : %i->MODE_APPROACH (%i)", __LINE__, mode,
 //        MODE_APPROACH);
 mode = MODE_INIT;
 return;
 }

 if (distance_to_despoint < RADIUS_APPROACH_ZONE and passe == true)
 {
 //ROS_INFO("ligne %i : %i->MODE_PASS (%i)", __LINE__, mode, MODE_PASS);
 mode = MODE_PASS;
 passe_time = ros::Time::now().toSec();
 ROS_INFO(">>>>>>>>>>>>>>>>> %s: Position Reached :  x=%.3f, y=%.3f, theta=%.3f", action_name_.c_str(),
 m_position.x(), m_position.y(), m_cap.angle());
 as_.setSucceeded(result_);
 endOrder = true;
 success = true;
 return;
 }

 }

 if (mode == MODE_PASS)
 {
 if (ros::Time::now().toSec() - passe_time > 1.0)
 {
 //ROS_INFO("ligne %i : %i->MODE_DONE (%i)", __LINE__, mode, MODE_DONE);
 mode = MODE_DONE;
 return;
 }

 }

 if (mode == MODE_INIT or mode == MODE_RUN)
 {
 //success ?
 if (distance_error < DISTANCE_ACCURACY && fabs(angle_error) < ANGLE_ACCURACY)
 {

 ROS_INFO(">>>>>>>>>>>>>>>>> %s: Position Reached :  x=%.3f, y=%.3f, theta=%.3f", action_name_.c_str(),
 m_position.x(), m_position.y(), m_cap.angle());

 //ROS_INFO("ligne %i : %i->MODE_DONE (%i)", __LINE__, mode, MODE_DONE);

 mode = MODE_DONE;
 // set the action state to succeeded
 as_.setSucceeded(result_);
 endOrder = true;
 success = true;
 return;
 }

 // check that preempt has not been requested by the clientcur_
 if (as_.isPreemptRequested() || !ros::ok())
 {
 ROS_INFO("%s: Preempted", action_name_.c_str());
 // set the action state to preempted
 as_.setPreempted();
 success = false;
 endOrder = true;

 //ROS_INFO("ligne %i : %i->MODE_DONE (%i)", __LINE__, mode, MODE_DONE);

 mode = MODE_DONE;
 return;
 }
 }
 }
 */

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

