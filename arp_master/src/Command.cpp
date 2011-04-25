#include "Command.hpp"

//FIXME 39 Passer ces paramètres en param ROS
#define DEFAULT_BASE_LINE 0.4   // distance between wheels
#define DEFAULT_WHEEL_DIAM 0.07 // wheel diameter

using namespace arp_master;

Command::Command():
    nh(),
    velocity_sub(),
    command_pub()
{
    velocity_sub = nh.subscribe("Command/velocity", 1, &Command::velocityCallback, this);
    command_pub = nh.advertise<DifferentialCommand>("Protokrot/differential_command", 1);
}

Command::~Command()
{
    ;
}

void Command::velocityCallback(const VelocityConstPtr& v)
{
    double lin_vel = v->linear;
    double ang_vel = v->angular;

    double v_right = (2.0 * lin_vel + DEFAULT_BASE_LINE * ang_vel ) / DEFAULT_WHEEL_DIAM;
    double v_left  = (2.0 * lin_vel - DEFAULT_BASE_LINE * ang_vel ) / DEFAULT_WHEEL_DIAM;
    
    DifferentialCommand c;
    c.v_left = v_left;
    c.v_right = v_right;
    command_pub.publish(c);

    //ROS_INFO("lin_vel=%f, ang_vel=%f, v_left=%f, v_right=%f", lin_vel, ang_vel, v_left, v_right);
}


