#include "Command.hpp"

using namespace arp_core;

using namespace arp_ods;

using namespace arp_math;

Command::Command() :
    velocity_sub(), command_pub()
{
    velocity_sub = nh.subscribe("Command/velocity", 1,
            &Command::velocityCallback, this);
    command_pub = nh.advertise<DifferentialCommand> (
            "Protokrot/differential_command", 1);

    nh.getParam("/arp_ods/LIN_ACC_MAX", LIN_ACC_MAX);
    nh.getParam("/arp_ods/LIN_DEC_MAX", LIN_DEC_MAX);
    nh.getParam("/arp_ods/ANG_ACC_MAX", ANG_ACC_MAX);

    nh.getParam("/Protokrot/BASE_LINE", BASE_LINE);
    nh.getParam("/Protokrot/WHEEL_DIAMETER", WHEEL_DIAMETER);


}

Command::~Command()
{
    ;
}

void Command::velocityCallback(const VelocityConstPtr& v)
{

    double lin_vel_cons_full = v->linear;
    double ang_vel_cons_full = v->angular;

    ////////////////////rampage des consignes

    double old_loop_date = loop_date;
    loop_date = ros::Time::now().toSec();
    double delta_date = loop_date - old_loop_date;

    double old_lin_vel = lin_vel_;
    double old_ang_vel = ang_vel_;

    double delta_lin_vel = saturate(
            (lin_vel_cons_full - old_lin_vel) / delta_date, LIN_DEC_MAX,
            LIN_ACC_MAX);
    double delta_ang_vel = saturate(
            (ang_vel_cons_full - old_ang_vel) / delta_date, -ANG_ACC_MAX,
            ANG_ACC_MAX);

    lin_vel_ = old_lin_vel + delta_lin_vel * delta_date;
    ang_vel_ = old_ang_vel + delta_ang_vel * delta_date;

    ////////////////creation consigne droite et consigne gauche
    // l'expression usuelle serait:    ( linvel + ang_vel*baseline/2  )  / wheel_radius
    double v_right = (2.0 * lin_vel_ + BASE_LINE * ang_vel_) / WHEEL_DIAMETER;
    double v_left = (2.0 * lin_vel_ - BASE_LINE * ang_vel_) / WHEEL_DIAMETER;

    DifferentialCommand c;
    c.v_left = v_left;
    c.v_right = v_right;
    command_pub.publish(c);

    //ROS_INFO("lin_vel=%f, ang_vel=%f, v_left=%f, v_right=%f", lin_vel_,
    //        ang_vel_, v_left, v_right);
}

