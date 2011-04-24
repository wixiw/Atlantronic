#include <ros/ros.h>
#include "TinyStrat.hpp"
        
using namespace arp_master;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Tiny_Strat");

  TinyStrat s;
  s.go();

  //exit
  return 0;
}

