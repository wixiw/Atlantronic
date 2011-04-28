#include <ros/ros.h>
#include <signal.h>
#include "PhysicsSimu.hpp"

using namespace arp_master;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Strat_Archi_A");

  PhysicsSimu s;

  //exit
  return 0;
}
