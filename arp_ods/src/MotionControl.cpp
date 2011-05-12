#include <tf/transform_datatypes.h>
#include "MotionControl.hpp"



using namespace arp_math;
using namespace arp_ods;
using namespace nav_msgs;
using namespace geometry_msgs;

MotionControl::MotionControl(std::string name) :
    as_(nh_, name, boost::bind(&MotionControl::executeCB, this, _1), false),
            action_name_(name),
            m_position(0.0,0.0),
            m_transCallback(0.0,0.0),
            m_cap(0.0),
            m_orientCallback(0.0),
            orientLocal_(0.0),
            m_linearSpeedCmd(0.0),
            m_angularSpeedCmd(0.0)

{
    nh_.getParam("/arp_ods/DISTANCE_ACCURACY", DISTANCE_ACCURACY);
    nh_.getParam("/arp_ods/ANGLE_ACCURACY", ANGLE_ACCURACY);
    nh_.getParam("/arp_ods/ROTATION_GAIN", ROTATION_GAIN);
    nh_.getParam("/arp_ods/TRANSLATION_GAIN", TRANSLATION_GAIN);
    nh_.getParam("/arp_ods/LIN_VEL_MAX", LIN_VEL_MAX);
    nh_.getParam("/arp_ods/ANG_VEL_MAX", ANG_VEL_MAX);
    nh_.getParam("/arp_ods/VEL_FINAL", VEL_FINAL);
    nh_.getParam("/arp_ods/RADIUS_APPROACH_ZONE", RADIUS_APPROACH_ZONE);
    nh_.getParam("/arp_ods/RADIUS_FANTOM_ZONE", RADIUS_FANTOM_ZONE);
    nh_.getParam("/arp_ods/FANTOM_COEF", FANTOM_COEF);

    // Suscribers
    pose_sub_ = nh_.subscribe("Localizator/odomRos", 1,
            &MotionControl::poseCallback, this);
    //TODO WLA test laserator
    //pose_sub_WLA = nh_.subscribe("pose2D", 1,
    //        &MotionControl::pose2DCallback, this);

    vel_pub_ = nh_.advertise<arp_core::Velocity> ("Command/velocity", 1);

    as_.start();



}

MotionControl::~MotionControl(void)
{
}

void MotionControl::poseCallback(OdometryConstPtr c)
{
	double x = c->pose.pose.position.x;
	double y = c->pose.pose.position.y;
	geometry_msgs::Quaternion thetaQuaternion = c->pose.pose.orientation;
	m_transCallback= Vector2(x, y);
    m_orientCallback = Rotation2(tf::getYaw(thetaQuaternion));
}

void MotionControl::pose2DCallback(Pose2DConstPtr c)
{
	ROS_WARN("Pose 2D ???");
	m_transCallback= Vector2(c->x, c->y);
    m_orientCallback = Rotation2(c->theta);
}

void MotionControl::executeCB(const OrderGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(100);
    bool success = true;
    bool loop = true;

    /////////////////////// Receiving goal

    double x_des;
    double y_des;
    double theta_des;

    if (goal->move_type == "POINTCAP")
    {
        // Standard move: go to a point, with a cap
        x_des = goal->x_des;
        y_des = goal->y_des;
        theta_des = goal->theta_des;
    }
    else if (goal->move_type == "CAP")
    {
        // only cap move
        x_des = m_position.x();
        y_des = m_position.y();
        theta_des = goal->theta_des;
    }
    else
    {
        ROS_INFO("%s: not possible", action_name_.c_str());
        // reject action
        as_.setAborted(result_);
        return;
    }

    Vector2 trans_des(goal->x_des, goal->y_des);
    Rotation2 orient_des(theta_des);

    ROS_INFO("%s: Receiving order :  x_des=%.3f, y_des=%.3f, theta_des=%.3f",
            action_name_.c_str(), trans_des.x(),
            trans_des.y(), theta_des);

    ////////////////////////// CONTROL LOOP
    // NB: the control loop has been made so that every "move type" (normal, only cap, reverse) and every step of the trajectory (far, approaching) is done by the same algorithm
    while (loop)
    {
        double sens_lin;
        //bufferisation des données reçues dans la callback pour la thread-safety
        m_position = m_transCallback;
        m_cap = m_orientCallback;
        //ROS_WARN("pos : %f %f cap : %f ", m_position.x(),m_position.y(),m_cap.angle());

        //permet de retourner le repere du robot pour le faire partir en marche arriere
        if (goal->reverse == false)
        {
            orientLocal_= m_cap;
            sens_lin = 1;
        }
        else
        {
            orientLocal_ = Rotation2(normalizeAngle(m_cap.angle()+PI));
            sens_lin = -1;
        }

        //ROS_INFO("*************************************************************");
        //ROS_INFO("m_position : x=%.3f, y=%.3f, theta=%.3f", m_position.x(), m_position.y(), m_cap.angle());

        ///////////////////////////////////////////////// DISTANCES
        // calcul de la distance au point desire
        double distance_to_despoint = (trans_des - m_position).norm();

        //calcul de la distance à la droite passant par le point desire et perpendiculaire à l'angle desire
        //la distance à la ligne d'arrivée en quelque sorte
        double distance_to_desline = (orient_des.inverse() * (trans_des
                - m_position)).x();

        //utilisation du barycentre entre les 2: quand on est loin, on regarde la distance au point
        //et quand on est pret on regarde la distance à la ligne
        double distance_error = smoothStep(distance_to_despoint,
                distance_to_desline, RADIUS_APPROACH_ZONE,
                distance_to_despoint, RADIUS_FANTOM_ZONE);

        //ROS_INFO(
        //       "distance_to_despoint=%.3f, distance_to_desline=%.3f, distance_error=%.3f",
        //        distance_to_despoint, distance_to_desline, distance_error);

        ///////////////////////////////////////////////// ANGLES
        //creation du point fantome
        //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
        //c'est lui qu'on va viser en cap
        Vector2 phantompoint = trans_des - FANTOM_COEF * distance_error
                * Vector2(cos(theta_des), sin(theta_des));
        //calcul de l'angle au point fantome
        Vector2 phantom = phantompoint - m_position;
        Vector2 direction_in_current = orientLocal_.inverse() * phantom;
        double angle_error_fant = atan2(direction_in_current.y(),
                direction_in_current.x());

        //ROS_INFO("fantom     : x=%.3f, y=%.3f", phantompoint.x(), phantompoint.y());

        //calcul de l'erreur d'angle par rapport a l'objectif
        double angle_error_des = normalizeAngle(
                orient_des.angle() - orientLocal_.angle());

        //utilisation du barycentre entre les 2: quand on est loin, on regarde l'erreur d'angle avec le point fantome
        //et quand on est pres, on regarde l'erreur avec l'angle final
        double angle_error = smoothStep(distance_to_despoint, angle_error_des,
                RADIUS_APPROACH_ZONE, angle_error_fant, RADIUS_FANTOM_ZONE);

        //ROS_INFO(
        //        "angle_error_fant=%.3f, angle_error_des=%.3f, angle_error=%.3f",
        //        angle_error_fant, angle_error_des, angle_error);

        //////////////////////////////////////CONSIGNES

        //creation des consignes full patates
        double lin_vel_cons_full = (TRANSLATION_GAIN * sqrt2(distance_error)
                * cos(angle_error) + VEL_FINAL) * sens_lin;
        double ang_vel_cons_full = ROTATION_GAIN * angle_error;

        //saturation des consignes
        m_linearSpeedCmd = saturate(lin_vel_cons_full, -LIN_VEL_MAX, LIN_VEL_MAX);
        m_angularSpeedCmd = saturate(ang_vel_cons_full, -ANG_VEL_MAX, ANG_VEL_MAX);

        //ROS_INFO("distance_error=%.3f angle_error=%.3f", distance_error,
        //        angle_error);

        //ROS_INFO("lin_vel=%f, ang_vel=%f", m_linearSpeedCmd, m_angularSpeedCmd);

        ////////////////////////////////////////ATTEINTE DU BUT
        if (distance_error < DISTANCE_ACCURACY && abs(angle_error)
                < ANGLE_ACCURACY)
        {
            ROS_INFO(">>>>>>>>>>>>>>>>> %s: Position Reached :  x_des=%.3f, y_des=%.3f, theta_des=%.3f",
                    action_name_.c_str(),m_position.x(), m_position.y(), m_cap.angle());
            success = true;
            break;
        }

        // check that preempt has not been requested by the clientcur_
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }

        /////////////////////// PUBLICATIONS
        // On publie la consigne
        Velocity vel;
        vel.linear = m_linearSpeedCmd;
        vel.angular = m_angularSpeedCmd;
        vel_pub_.publish(vel);

        // On publie le feedback
        feedback_.x_current = m_position.x();
        feedback_.y_current = m_position.y();
        feedback_.theta_current = m_cap.angle();
        feedback_.lin_vel = m_linearSpeedCmd;
        feedback_.ang_vel = m_angularSpeedCmd;
        as_.publishFeedback(feedback_);

        r.sleep();
        ros::spinOnce();
    }

    if (success)
    {
        result_.x_end = feedback_.x_current;
        result_.y_end = feedback_.y_current;
        result_.theta_end = feedback_.theta_current;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        ROS_INFO("%s: Waiting for a new order", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

