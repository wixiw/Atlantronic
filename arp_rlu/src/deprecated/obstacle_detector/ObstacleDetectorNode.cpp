#include "ros/ros.h"

#include "ObstacleDetector.hpp"

using namespace arp_rlu;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObstacleDetector");

  ObstacleDetector obstacleDetector;

  obstacleDetector.go();

  return 0;

}

