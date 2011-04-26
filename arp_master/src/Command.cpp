#include "Command.hpp"

using namespace arp_core;

using namespace arp_master;

Command::Command():
nh(),
velocity_sub(),
command_pub()
{
velocity_sub = nh.subscribe("Command/velocity", 1, &Command::velocityCallback, this);
command_pub = nh.advertise<DifferentialCommand>("Protokrot/differential_command", 1);

if( nh.getParam("/Protokrot/base_line", base_line))
{
  ROS_INFO("Got param named '/Protokrot/base_line' : %f", base_line);
}
else
{
  ROS_ERROR("Failed to get param '/Protokrot/base_line'. Take default value (0.4)");
  base_line = 0.4;
}

if( nh.getParam("/Protokrot/wheel_diameter", wheel_diameter))
{
  ROS_INFO("Got param named '/Protokrot/wheel_diameter' : %f", wheel_diameter);
}
else
{
  ROS_ERROR("Failed to get param '/Protokrot/wheel_diameter'. Take default value (0.07)");
  wheel_diameter = 0.07;
}
}

Command::~Command()
{
;
}

void Command::velocityCallback(const VelocityConstPtr& v)
{
double lin_vel = v->linear;
double ang_vel = v->angular;

double v_right = (2.0 * lin_vel + base_line * ang_vel ) / wheel_diameter;
double v_left  = (2.0 * lin_vel - base_line * ang_vel ) / wheel_diameter;

DifferentialCommand c;
c.v_left = v_left;
c.v_right = v_right;
command_pub.publish(c);

//ROS_INFO("lin_vel=%f, ang_vel=%f, v_left=%f, v_right=%f", lin_vel, ang_vel, v_left, v_right);
}


