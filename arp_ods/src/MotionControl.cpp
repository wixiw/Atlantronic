#include <tf/transform_datatypes.h>
#include "MotionControl.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace nav_msgs;
using namespace geometry_msgs;

MotionControl::MotionControl(std::string name) :
    as_(nh_, name, boost::bind(&MotionControl::executeCB, this, _1), false),
            action_name_(name), m_position(0.0, 0.0),
            m_transCallback(0.0, 0.0), m_cap(0.0), m_orientCallback(0.0),
            orientLocal_(0.0), orient_desLocal_(0.0), m_linearSpeedCmd(0.0),
            m_angularSpeedCmd(0.0), loop_date(0.0), mode(MODE_FANTOM)

{
    if (nh_.getParam("/arp_ods/DISTANCE_ACCURACY", DISTANCE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre DISTANCE_ACCURACY");
    if (nh_.getParam("/arp_ods/ANGLE_ACCURACY", ANGLE_ACCURACY) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANGLE_ACCURACY");
    if (nh_.getParam("/arp_ods/ROTATION_GAIN", ROTATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_GAIN");
    if (nh_.getParam("/arp_ods/ROTATION_D_GAIN", ROTATION_D_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ROTATION_D_GAIN");
    if (nh_.getParam("/arp_ods/TRANSLATION_GAIN", TRANSLATION_GAIN) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre TRANSLATION_GAIN");
    if (nh_.getParam("/arp_ods/LIN_VEL_MAX", LIN_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre LIN_VEL_MAX");
    if (nh_.getParam("/arp_ods/ANG_VEL_MAX", ANG_VEL_MAX) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre ANG_VEL_MAX");
    if (nh_.getParam("/arp_ods/VEL_FINAL", VEL_FINAL) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre VEL_FINAL");
    if (nh_.getParam("/arp_ods/RADIUS_APPROACH_ZONE", RADIUS_APPROACH_ZONE)
            == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RADIUS_APPROACH_ZONE");
    if (nh_.getParam("/arp_ods/RADIUS_FANTOM_ZONE", RADIUS_FANTOM_ZONE) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RADIUS_FANTOM_ZONE");
    if (nh_.getParam("/arp_ods/FANTOM_COEF", FANTOM_COEF) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre FANTOM_COEF");

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
    m_transCallback = Vector2(x, y);
    m_orientCallback = Rotation2(tf::getYaw(thetaQuaternion));
}

void MotionControl::pose2DCallback(Pose2DConstPtr c)
{
    ROS_WARN("Pose 2D ???");
    m_transCallback = Vector2(c->x, c->y);
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
        mode = MODE_FANTOM;
    }
    else if (goal->move_type == "CAP")
    {
        // only cap move
        x_des = m_position.x();
        y_des = m_position.y();
        theta_des = goal->theta_des;
        mode = MODE_APPROACH;
    }
    else
    {
        ROS_INFO("%s: not possible", action_name_.c_str());
        // reject action
        as_.setAborted(result_);
        return;
    }

    Vector2 trans_des(x_des, y_des);
    Rotation2 orient_des(theta_des);

    ROS_INFO("%s: Receiving order :  x_des=%.3f, y_des=%.3f, theta_des=%.3f",
            action_name_.c_str(), trans_des.x(), trans_des.y(), theta_des);
    if (goal->reverse == false)
        ROS_INFO("en normal");
    else
        ROS_INFO("en reverse");

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
            orientLocal_ = m_cap;
            orient_desLocal_ = orient_des;
            sens_lin = 1;
        }
        else
        {
            orientLocal_ = Rotation2(normalizeAngle(m_cap.angle() + PI));
            orient_desLocal_ = Rotation2(
                    normalizeAngle(orient_des.angle() + PI));
            sens_lin = -1;
        }

        //ROS_INFO("*************************************************************");
        //ROS_INFO("m_position : x=%.3f, y=%.3f, theta=%.3f", m_position.x(), m_position.y(), m_cap.angle());

        ///////////////////////////////////////////////// DISTANCES
        // calcul de la distance au point desire
        double distance_to_despoint = (trans_des - m_position).norm();

        if (distance_to_despoint < RADIUS_APPROACH_ZONE)
            mode = MODE_APPROACH;

        //calcul de la distance à la droite passant par le point desire et perpendiculaire à l'angle desire
        //la distance à la ligne d'arrivée en quelque sorte
        double distance_to_desline = (orient_desLocal_.inverse() * (trans_des
                - m_position)).x();

        /* ca partait d'un bonne intention mais je reviens a la vieille methode...
         //utilisation du barycentre entre les 2: quand on est loin, on regarde la distance au point
         //et quand on est pret on regarde la distance à la ligne
         double distance_error = smoothStep(distance_to_despoint,
         distance_to_desline, RADIUS_APPROACH_ZONE,
         distance_to_despoint, RADIUS_FANTOM_ZONE);
         */
        double distance_error;
        if (mode == MODE_FANTOM)
            distance_error = distance_to_despoint;
        if (mode == MODE_APPROACH)
            distance_error = distance_to_desline;

        //ROS_INFO("mode:%i",mode);

        //ROS_INFO(
        //       "distance_to_despoint=%.3f, distance_to_desline=%.3f, distance_error=%.3f",
        //        distance_to_despoint, distance_to_desline, distance_error);

        ///////////////////////////////////////////////// ANGLES
        //creation du point fantome
        //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
        //c'est lui qu'on va viser en cap
        Vector2 phantompoint = trans_des - FANTOM_COEF * distance_error
                * Vector2(cos(orient_desLocal_.angle()),
                        sin(orient_desLocal_.angle()));
        //calcul de l'angle au point fantome
        Vector2 phantom = phantompoint - m_position;
        Vector2 direction_in_current = orientLocal_.inverse() * phantom;
        double angle_error_fant = atan2(direction_in_current.y(),
                direction_in_current.x());

        //ROS_INFO("fantom     : x=%.3f, y=%.3f", phantompoint.x(), phantompoint.y());

        //calcul de l'erreur d'angle par rapport a l'objectif
        double angle_error_des = normalizeAngle(
                orient_desLocal_.angle() - orientLocal_.angle());

        /* j'ai retire le smoothstep
         *
         //utilisation du barycentre entre les 2: quand on est loin, on regarde l'erreur d'angle avec le point fantome
         //et quand on est pres, on regarde l'erreur avec l'angle final
         double angle_error = smoothStep(distance_to_despoint, angle_error_des,
         RADIUS_APPROACH_ZONE, angle_error_fant, RADIUS_FANTOM_ZONE);
         */
        double angle_error;
        if (mode == MODE_FANTOM)
            angle_error = angle_error_fant;
        if (mode == MODE_APPROACH)
            angle_error = angle_error_des;

        //ROS_INFO(
        //        "angle_error_fant=%.3f, angle_error_des=%.3f, angle_error=%.3f",
        //        angle_error_fant, angle_error_des, angle_error);

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
        double lin_vel_cons_full = (TRANSLATION_GAIN * sqrt2(distance_error)
                * linearReductionCoef(angle_error) + VEL_FINAL) * sens_lin;
        double ang_vel_cons_full = ROTATION_GAIN * angle_error;

        //ROS_INFO("angle_error=%.3f d_angle_error=%.3f", angle_error,
        //		d_angle_error);

        //saturation des consignes
        m_linearSpeedCmd = saturate(lin_vel_cons_full, -LIN_VEL_MAX,
                LIN_VEL_MAX);
        m_angularSpeedCmd = saturate(ang_vel_cons_full, -ANG_VEL_MAX,
                ANG_VEL_MAX);

        //ROS_INFO("distance_error=%.3f angle_error=%.3f", distance_error,
        //        angle_error);

        //ROS_INFO("lin_vel=%f, ang_vel=%f", m_linearSpeedCmd, m_angularSpeedCmd);

        ////////////////////////////////////////ATTEINTE DU BUT
        if (distance_error < DISTANCE_ACCURACY && d_abs(angle_error)
                < ANGLE_ACCURACY)
        {
            ROS_INFO(
                    ">>>>>>>>>>>>>>>>> %s: Position Reached :  x=%.3f, y=%.3f, theta=%.3f",
                    action_name_.c_str(), m_position.x(), m_position.y(),
                    m_cap.angle());
            /*ROS_INFO("erreur dist: %.3f   erreur angle: %.3f", distance_error,
             angle_error);
             ROS_INFO("angle_error_fant: %.3f   angle_error_des: %.3f",
             angle_error_fant, angle_error_des);
             ROS_INFO("orient_des : %.3f    orient  : %.3f", orient_des.angle(),
             m_cap.angle());
             ROS_INFO(
             "orient_desLocal_.angle() : %.3f    orientLocal_.angle()  : %.3f ==> angle_error_des: %.3f",
             orient_desLocal_.angle(), orientLocal_.angle(),
             angle_error_des);*/
            /*ROS_INFO("----");
             ROS_INFO(
             "double angle_error = smoothStep(distance_to_despoint, angle_error_des,RADIUS_APPROACH_ZONE, angle_error_fant, RADIUS_FANTOM_ZONE)");
             ROS_INFO("distance_to_despoint %.3f", distance_to_despoint);
             ROS_INFO("angle_error_des %.3f", angle_error_des);
             ROS_INFO("RADIUS_APPROACH_ZONE %.3f", RADIUS_APPROACH_ZONE);
             ROS_INFO("angle_error_fant %.3f", angle_error_fant);
             ROS_INFO("RADIUS_FANTOM_ZONE %.3f", RADIUS_FANTOM_ZONE);
             ROS_INFO(">>angle_error %.3f", angle_error);
             */
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

double MotionControl::linearReductionCoef(double angle_error)
{
    //le coefficient est 1 si angle est =0, il est 0 si angle=PI/8 au +, avec un smoothstep entre 2
    double result = smoothStep(d_abs(angle_error), 1, 0, 0, PI / 8.0);
    //ROS_INFO("smoothStep  error %.3f  =>> abs %.3f ",angle_error,d_abs(angle_error));
    //ROS_INFO("smoothStep abs error %.3f  =>> coef %.3f ",d_abs(angle_error), result);
    return result;
}

