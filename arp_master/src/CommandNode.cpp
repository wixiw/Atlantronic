#include "ros/ros.h"

#include "Command.hpp"

using namespace arp_master;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Command");

  Command com;

  ros::spin();

  return 0;

}



