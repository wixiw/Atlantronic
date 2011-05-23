#include <tf/transform_datatypes.h>
#include "MotionControl.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace nav_msgs;
using namespace geometry_msgs;

MotionControl::MotionControl(std::string name) :
    as_(nh_, name, boost::bind(&MotionControl::newOrderCB, this, _1), false),
            action_name_(name), m_position(0.0, 0.0), m_cap(0.0),
            orientLocal_(0.0), orient_desLocal_(0.0), m_linearSpeedCmd(0.0),
            m_angularSpeedCmd(0.0), loop_date(0.0), mode(MODE_DONE),
            trans_des(0, 0), orient_des(0), x_des(0), y_des(0), theta_des(0),
            success(false), loop(true), sens_lin(1), distance_to_despoint(0),
            reverse(false), distance_error(0), angle_error(0), endOrder(true),
            passe(false), passe_time(0)

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
    m_position = Vector2(x, y);
    m_cap = Rotation2(tf::getYaw(thetaQuaternion));

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

/////////////////////////// MAIN LOOP//////////////////////
//this is the main loop of the motion control.
void MotionControl::execute()
{
    ros::Rate r(100);
    while (ros::ok())
    {
        // depile les callback. la queue n'est qu'un d'1, donc chaque callback ne doit etre executée qu'une fois (la plus recente)
        // permet de mettre a jour la pose, les nouveaux ordres:
        // newOrderCB() va etre potentiellement appelée si nouvel ordre
        // poseCallback() va etre appelée une fois si la position a été mise a jour
        //ROS_INFO("avant spin: mode %i",mode);
        ros::spinOnce();

        //ROS_INFO("m_position : x=%.3f, y=%.3f, theta=%.3f", m_position.x(), m_position.y(), m_cap.angle());

        /*
         double old_t;
         double t = ros::Time::now().toSec() * 1000;
         ROS_INFO("MOTION: %.1f", t - old_t);
         old_t = t;
         */
        //ROS_INFO("avant process: mode %i",mode);

        // calcule les consignes
        processMotion();
        //change d'etat si necessaire
        //ROS_INFO("avant switch: mode %i",mode);
        switchState();
        // publie les consignes
        //ROS_INFO("avant publish: mode %i",mode);
        publish();
        //ROS_INFO("avant sleep: mode %i",mode);
        //dort le temps necessaire pour que l'execution de la boucle soit cyclique
        r.sleep();
    }
}
/////////////////////////////////////////////////////////

double MotionControl::linearReductionCoef(double angle_error)
{
    //le coefficient est 1 si angle est =0, il est 0 si angle=PI/8 au +, avec un smoothstep entre 2
    double result = smoothStep(fabs(angle_error), 1, 0, 0, PI / 8.0);
    //ROS_INFO("smoothStep  error %.3f  =>> abs %.3f ",angle_error,d_abs(angle_error));
    //ROS_INFO("smoothStep abs error %.3f  =>> coef %.3f ",d_abs(angle_error), result);
    return result;
}

void MotionControl::newOrderCB(const OrderGoalConstPtr &goal)
{
    //Receiving goal

    theta_des = goal->theta_des;
    reverse = goal -> reverse;
    passe = goal->passe;

    if (goal->move_type == "POINTCAP")
    {
        // Standard move: go to a point, with a cap
        x_des = goal->x_des;
        y_des = goal->y_des;

        //ROS_INFO("ligne %i : %i->MODE_FANTOM (%i)", __LINE__, mode, MODE_FANTOM);

        mode = MODE_FANTOM;

    }
    else if (goal->move_type == "CAP" and goal->passe == false)
    {
        // only cap move
        x_des = m_position.x();
        y_des = m_position.y();

        //ROS_INFO("ligne %i : %i->MODE_APPROACH (%i)", __LINE__, mode,
        //        MODE_APPROACH);

        mode = MODE_APPROACH;
    }
    else
    {
        ROS_INFO("%s: not possible", action_name_.c_str());
        // reject action
        as_.setAborted(result_);
        return;
    }

    trans_des = Vector2(x_des, y_des);
    orient_des = Rotation2(theta_des);

    ROS_INFO("%s: Receiving order :  x_des=%.3f, y_des=%.3f, theta_des=%.3f",
            action_name_.c_str(), trans_des.x(), trans_des.y(), theta_des);

    // thanks to the magnificence of ROS, I shall not get out of this callback until the order is fullfilled
    // in fact, its name is expected to be "execute CB" and not "new order CB"
    ros::Rate r(100);
    endOrder = false;
    while (endOrder == false)
    {
        r.sleep();
    }
}

void MotionControl::processMotion()
{

    if (mode == MODE_PASS)
    {
        return;
    }

    if (mode == MODE_DONE)
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
    double distance_to_desline = (orient_desLocal_.inverse() * (trans_des
            - m_position)).x();

    // selon le mode, je regarde la distance au point ou a la droite
    if (mode == MODE_FANTOM)
        distance_error = distance_to_despoint;
    if (mode == MODE_APPROACH)
        distance_error = distance_to_desline;

    ///////////////////////////////////////////////// ANGLES
    //creation du point fantome
    //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
    //c'est lui qu'on va viser en cap
    Vector2 phantompoint = trans_des - FANTOM_COEF * distance_error * Vector2(
            cos(orient_desLocal_.angle()), sin(orient_desLocal_.angle()));
    //calcul de l'angle au point fantome
    Vector2 phantom = phantompoint - m_position;
    Vector2 direction_in_current = orientLocal_.inverse() * phantom;
    double angle_error_fant = atan2(direction_in_current.y(),
            direction_in_current.x());

    //calcul de l'erreur d'angle par rapport a l'objectif
    double angle_error_des = normalizeAngle(
            orient_desLocal_.angle() - orientLocal_.angle());

    if (mode == MODE_FANTOM)
        angle_error = angle_error_fant;
    if (mode == MODE_APPROACH)
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
    double lin_vel_cons_full = (TRANSLATION_GAIN * sqrt2(distance_error)
            * linearReductionCoef(angle_error) + VEL_FINAL) * sens_lin;
    double ang_vel_cons_full = ROTATION_GAIN * angle_error;

    //saturation des consignes
    m_linearSpeedCmd = saturate(lin_vel_cons_full, -LIN_VEL_MAX, LIN_VEL_MAX);
    m_angularSpeedCmd = saturate(ang_vel_cons_full, -ANG_VEL_MAX, ANG_VEL_MAX);

}

void MotionControl::switchState()
{
    //approaching ??
    if (mode == MODE_FANTOM)
    {
        if (distance_to_despoint < RADIUS_APPROACH_ZONE and passe == false)
        {
            //ROS_INFO("ligne %i : %i->MODE_APPROACH (%i)", __LINE__, mode,
            //       MODE_APPROACH);
            mode = MODE_APPROACH;
            return;
        }

        if (distance_to_despoint < RADIUS_APPROACH_ZONE and passe == true)
        {
            //ROS_INFO("ligne %i : %i->MODE_PASS (%i)", __LINE__, mode, MODE_PASS);
            mode = MODE_PASS;
            passe_time = ros::Time::now().toSec();
            ROS_INFO(
                    ">>>>>>>>>>>>>>>>> %s: Position Reached :  x=%.3f, y=%.3f, theta=%.3f",
                    action_name_.c_str(), m_position.x(), m_position.y(),
                    m_cap.angle());
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

    if (mode == MODE_APPROACH or mode == MODE_FANTOM)
    {
        //success ?
        if (distance_error < DISTANCE_ACCURACY && fabs(angle_error)
                < ANGLE_ACCURACY)
        {

            ROS_INFO(
                    ">>>>>>>>>>>>>>>>> %s: Position Reached :  x=%.3f, y=%.3f, theta=%.3f",
                    action_name_.c_str(), m_position.x(), m_position.y(),
                    m_cap.angle());

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

void MotionControl::publish()
{

    /////////////////////// PUBLICATIONS
    // On publie la consigne
    Velocity vel;
    vel.linear = m_linearSpeedCmd;
    vel.angular = m_angularSpeedCmd;
    vel_pub_.publish(vel);

}
