#include "ros/ros.h"

#include "OldLocalizator.hpp"

using namespace arp_rlu;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OldLocalizator");

  OldLocalizator loc;

  ros::spin();

  return 0;

}

