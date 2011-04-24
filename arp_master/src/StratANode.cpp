#include <ros/ros.h>
#include "StratA.hpp"

using namespace arp_master;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Strat_Archi_A");

  StratA s;
  s.initTraj();
  s.waitForServer();
  s.go();

  ros::spin();

  //exit
  return 0;
}

