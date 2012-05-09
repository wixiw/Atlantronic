#include "ros/ros.h"

#include "Localizator.hpp"

using namespace arp_rlu;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Localizator");

  Localizator loc;

  ros::spin();

  return 0;

}
