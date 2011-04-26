#include <ros/ros.h>
#include <signal.h>

#include "StratA.hpp"

using namespace arp_master;

StratA s;

void quit(int sig)
{
  s.shutDown();
  ros::shutdown();
  exit(0);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Strat_Archi_A");

  signal(SIGTERM,quit);

  s.initTraj();
  s.waitForServer();
  s.go();

  //exit
  return 0;
}

