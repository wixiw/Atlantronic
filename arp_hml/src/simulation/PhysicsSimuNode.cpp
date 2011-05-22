/*
 * PhysicsSimuNode
 *
 *  Created on: may 2011
 *      Author: bmo
 */

#include <ros/ros.h>
#include <signal.h>
#include "PhysicsSimu.hpp"

using namespace arp_hml;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Physics_Simu_Node");

  PhysicsSimu s;
  s.spawnRobot(0.0, 0.0, 0.0);

  ros::Rate r(100);
  ROS_INFO("Physics_Simu_Node is started");
  while (ros::ok())
  {

    s.updateRobot();
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Physics_Simu_Node is shutting down");
  ros::shutdown();

  //exit
  return 0;
}
