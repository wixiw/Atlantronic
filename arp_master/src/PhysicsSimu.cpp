#include "PhysicsSimu.hpp"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>


using namespace arp_master;

PhysicsSimu::PhysicsSimu()
: nh_(ros::NodeHandle("ARDPhysicsSimu"))
{
  srand(time(NULL));

  /*update_timer_ = new wxTimer(this);
  update_timer_->Start(default_dt_ms);
  Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(PhysicsSimu::onUpdate), NULL, this);*/

  respawn_srv_ = nh_.advertiseService("respawn", &PhysicsSimu::respawnCallback, this);

  ROS_INFO("Starting PhysicsSimu with node name %s", ros::this_node::getName().c_str()) ;

  spawnRobot(0.0, 0.0, 0.0);
}

PhysicsSimu::~PhysicsSimu()
{
}

bool PhysicsSimu::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
  ROS_INFO("Respawing ARDSimu to x=%f, y=%f and theta=%f", req.x, req.y, req.theta);
  mRobot.reset();  //"deleting" shared_ptr
  spawnRobot(req.x, req.y, req.theta);
  return true;
}


void PhysicsSimu::spawnRobot(double x, double y, double angle)
{
  SimuRobotPtr t(new SimuRobot(ros::NodeHandle("Protokrot"), Vector2(x, y), angle, one_meter_in_pixel));
  mRobot = t;

  ROS_INFO("Spawning robot [%s] at x=[%f], y=[%f], theta=[%f]", "Protokrot", x, y, angle);

  return;
}


void PhysicsSimu::onUpdate()
{
  ros::spinOnce();

  updateRobot();

}

void PhysicsSimu::updateRobot()
{
  if (last_robot_update_.isZero())
  {
    last_robot_update_ = ros::WallTime::now();
    return;
  }

  ros::WallTime t = ros::WallTime::now();
  double dt = (t - last_robot_update_).toSec();
  last_robot_update_ = t;
  mRobot->update(dt, 
                 table_length_in_pixel/one_meter_in_pixel, 
                 table_width_in_pixel/one_meter_in_pixel );

}



