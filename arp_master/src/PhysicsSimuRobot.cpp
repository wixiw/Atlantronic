#include "PhysicsSimuRobot.hpp"


using namespace arp_core;

using namespace arp_master;


PhysicsSimuRobot::PhysicsSimuRobot(const ros::NodeHandle& nh, const Vector2& pos, double orient)
: nh_(nh)
, pos_(pos)
, orient_(orient)
, v_left_(0.0)
, v_right_(0.0)
, lin_vel_(0.0)
, ang_vel_(0.0)
, odo_left_(0.0)
, odo_right_(0.0)
{

  // Parameters
  base_line      = 0.4;
  wheel_diameter = 0.07;
  nh_.setParam("/Protokrot/base_line", base_line);
  nh_.setParam("/Protokrot/wheel_diameter", wheel_diameter);

  // Suscribers
  differential_command_sub_ = nh_.subscribe("differential_command", 1, &PhysicsSimuRobot::commandCallback, this);

  // Publishers
  pose_pub_ = nh_.advertise<arp_core::Pose>("pose", 1);
  odo_pub_ = nh_.advertise<arp_core::Odo>("odo", 1);

}


void PhysicsSimuRobot::commandCallback(const arp_core::DifferentialCommandConstPtr& c)
{
  last_command_time_ = ros::WallTime::now();
  v_left_ = c->v_left;
  v_right_ = c->v_right;
}


void PhysicsSimuRobot::update(double dt, double canvas_width, double canvas_height)
{
  // Maintient de la commande pendant 0.2 seconde
  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(0.2))
  {
    v_right_ = 0.0f;
    v_left_ = 0.0f;
  }

  // Calcul du Twist
  lin_vel_ = 0.25 * ( v_right_ + v_left_ ) * wheel_diameter;
  ang_vel_ = 0.50 * ( v_right_ - v_left_ ) * wheel_diameter / base_line;


  // Calcul de la position réelle (integration)
  orient_ = orient_ * Rotation2(ang_vel_ * dt);
  Vector2 delta_trans = lin_vel_ * dt * Vector2(1.0, 0.0);
  pos_ += orient_ * delta_trans;

  
  // les vitesses odo valent la valeur des consignes
  double lin_vel_odo = lin_vel_;
  double ang_vel_odo = ang_vel_;

  // On simule des odo (ils sont suposés parfaits)
  double v_left_odo  = (2.0 * lin_vel_odo + base_line * ang_vel_odo ) / wheel_diameter;
  double v_right_odo = (2.0 * lin_vel_odo - base_line * ang_vel_odo ) / wheel_diameter;
  odo_right_ += v_left_odo  * dt;
  odo_left_  += v_right_odo * dt;
  

  // Clamp to screen size
  if (pos_.x() < -canvas_width/2.0 || pos_.x() >= canvas_width/2.0
      || pos_.y() < -canvas_height/2.0 || pos_.y() >= canvas_height/2.0)
  {
    ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  pos_.x() = std::min(std::max(pos_.x(), -canvas_width/(double)2.0), canvas_width/(double)2.0);
  pos_.y() = std::min(std::max(pos_.y(), -canvas_height/(double)2.0), canvas_height/(double)2.0);


  // Publishing
  Pose p;
  p.x = pos_.x();
  p.y = pos_.y();
  p.theta = fmod(orient_.angle(), 2*PI);
  p.linear_velocity = lin_vel_;
  p.angular_velocity = ang_vel_;
  pose_pub_.publish(p);

  Odo o;
  o.odo_left = odo_left_;
  o.odo_right = odo_right_;
  odo_pub_.publish(o);

  ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f", nh_.getNamespace().c_str(), pos_.x(), pos_.y(), orient_.angle());

}


