#include "MotionControl.hpp"

using namespace arp_math;
using namespace arp_master;

MotionControl::MotionControl(std::string name) :
            as_(nh_, name,
                    boost::bind(&MotionControl::executeCB, this, _1), false),
            action_name_(name), trans_(0.0, 0.0), orient_(0.0),
            lin_vel_(0.0), ang_vel_(0.0), distance_accuracy_(0.01),
            angle_accuracy_(0.05), rotation_gain_(5.0),
            translation_gain_(1.0)
{
    // Suscribers
    pose_sub_ = nh_.subscribe("Localizator/pose", 1,
            &MotionControl::poseCallback, this);
    vel_pub_ = nh_.advertise<arp_core::Velocity> ("Command/velocity", 1);

    as_.start();
}

MotionControl::~MotionControl(void)
{
}

void MotionControl::poseCallback(const PoseConstPtr& c)
{
    trans_ = Vector2(c->x, c->y);
    orient_ = Rotation2(c->theta);
}

void MotionControl::executeCB(const OrderGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(100);
    bool success = true;
    bool loop = true;

    // Receiving goal
    Vector2 trans_des(goal->x_des, goal->y_des);
    double angle_des = goal->theta_des;
    Rotation2 orient_des(angle_des);

    ROS_INFO("%s: Receiving order : x_des=%.3f, y_des=%.3f, theta_des=%.3f",
            action_name_.c_str(), trans_des.x(), trans_des.y(), angle_des);

    // start executing the action
    while (loop)
    {
        //ROS_INFO("*************************************************************");
        //ROS_INFO("position : x=%.3f, y=%.3f, theta=%.3f", trans_.x(), trans_.y(), orient_.angle());

        ///////////////////////////////////////////////// DISTANCES
        // calcul de la distance au point desire
        double distance_to_despoint = (trans_des - trans_).norm();

        //calcul de la distance à la droite passant par le point desire et perpendiculaire à l'angle desire
        //la distance à la ligne d'arrivée en quelque sorte
        double distance_to_desline = (orient_des.inverse() * (trans_des
                - trans_)).x();

        //utilisation du barycentre entre les 2: quand on est loin, on regarde la distance au point
        //et quand on est pret on regarde la distance à la ligne
        double distance_error = smoothStep(distance_to_despoint,
                distance_to_desline, RADIUS_APPROACH_ZONE, distance_to_despoint, RADIUS_FANTOM_ZONE);

        //ROS_INFO(
        //        "distance_to_despoint=%.3f, distance_to_desline=%.3f, distance_error=%.3f",
        //        distance_to_despoint, distance_to_desline, distance_error);

        ///////////////////////////////////////////////// ANGLES
        //creation du point fantome
        //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
        //c'est lui qu'on va viser en cap
        Vector2 phantompoint = trans_des - FANTOM_COEF * distance_error * Vector2(
                cos(angle_des), sin(angle_des));
        //calcul de l'angle au point fantome
        Vector2 phantom = phantompoint - trans_;
        Vector2 direction_in_current = orient_.inverse() * phantom;
        double angle_error_fant = atan2(direction_in_current.y(),
                direction_in_current.x());

        //ROS_INFO("fantom     : x=%.3f, y=%.3f", phantompoint.x(), phantompoint.y());

        //calcul de l'erreur d'angle par rapport a l'objectif
        double angle_error_des = normalizeAngle(
                orient_des.angle() - orient_.angle());

        //utilisation du barycentre entre les 2: quand on est loin, on regarde l'erreur d'angle avec le point fantome
        //et quand on est pres, on regarde l'erreur avec l'angle final
        double angle_error = smoothStep(distance_to_despoint,
                angle_error_des, RADIUS_APPROACH_ZONE, angle_error_fant, RADIUS_FANTOM_ZONE);

        //ROS_INFO(
        //        "angle_error_fant=%.3f, angle_error_des=%.3f, angle_error=%.3f",
        //        angle_error_fant, angle_error_des, angle_error);

        //////////////////////////////////////CONSIGNES

        //creation des consignes full patates
        double lin_vel_cons_full = translation_gain_
                * sqrt2(distance_error) * cos(angle_error) + VEL_FINAL;
        double ang_vel_cons_full = rotation_gain_ * angle_error;

        //saturation des consignes
        lin_vel_cons_full = saturate(lin_vel_cons_full, -LIN_VEL_MAX, LIN_VEL_MAX);
        ang_vel_cons_full = saturate(ang_vel_cons_full, -ANG_VEL_MAX, ANG_VEL_MAX);

        //rampage des consignes
        double old_lin_vel = lin_vel_;
        double old_ang_vel = ang_vel_;


        double delta_lin_vel = saturate(lin_vel_cons_full - old_lin_vel,
                -LIN_ACC_MAX, LIN_ACC_MAX);
        double delta_ang_vel = saturate(ang_vel_cons_full - old_ang_vel,
                -ANG_ACC_MAX, ANG_ACC_MAX);

        lin_vel_ = old_lin_vel + delta_lin_vel;
        ang_vel_ = old_ang_vel + delta_ang_vel;

        //ROS_INFO("distance_error=%.3f angle_error=%.3f", distance_error,
        //        angle_error);
        //ROS_INFO("lin_vel=%f, ang_vel=%f", lin_vel_, ang_vel_);

        ////////////////////////////////////////ATTEINTE DU BUT
        if (distance_error < distance_accuracy_ && abs(angle_error)
                < angle_accuracy_)
        {
            ROS_INFO(">>>>>>>>>>>>>>>>> %s: Position Reached",
                    action_name_.c_str());
            //Velocity vel;
            //vel.angular = 0.0;
            //vel.linear = 0.0;
            //vel_pub_.publish(vel);
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
        vel.linear = lin_vel_;
        vel.angular = ang_vel_;
        vel_pub_.publish(vel);

        // On publie le feedback
        feedback_.x_current = trans_.x();
        feedback_.y_current = trans_.y();
        feedback_.theta_current = orient_.angle();
        feedback_.lin_vel = lin_vel_;
        feedback_.ang_vel = ang_vel_;
        as_.publishFeedback(feedback_);

        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
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

double MotionControl::normalizeAngle(double angle)
{
    angle = fmod(angle, 2 * PI);
    if (angle > PI)
        angle = angle - 2 * PI;
    if (angle < -PI)
        angle = angle + 2 * PI;
    return angle;
}

double MotionControl::saturate(double value, double min, double max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

double MotionControl::sqrt2(double value)
{
    if (value > 0)
        return sqrt(value);
    if (value < 0)
        return -sqrt(-value);
    return 0;
}

double MotionControl::smoothStep(double x, double startValue,
        double startLimit, double endValue, double endLimit)
{
    //il s'agit d'une fonction toute simple qui permet de trouver la valeur intermediaire entre deux valeurs corresponsdant à des modes de calculs différents
    // la fonction est définir par
    // f(x) = startValue pour x <= startLimit
    // f(x) = un barycentrentre entre startValue et endValue pour  startLimit<x<endLimit
    // f(x) = endValue pour x>= endLimit
    if (x <= startLimit)
    {
        return startValue;
    }
    if (x > startLimit && x < endLimit)
    {
        //le barycentre
        return (startValue * (endLimit - x) + endValue * (x - startLimit))
                / (endLimit - startLimit);
    }
    if (x >= endLimit)
    {
        return endValue;
    }

    return 0;

}
